//******************************************************************************
// See LICENSE.Berkeley for license details.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

package boom.v4.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.tile._
import freechips.rocketchip.util._
import freechips.rocketchip.rocket._

import boom.v4.common._
import boom.v4.exu.BrResolutionInfo
import boom.v4.util._



abstract class DataPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends BoomModule()(p)
{
  val io = IO(new Bundle {
    val mshr_avail = Input(Bool())
    val req_val    = Input(Bool())
    val req_paddr  = Input(UInt(coreMaxAddrBits.W))
    val req_vaddr  = Input(UInt(coreMaxAddrBits.W))
    val req_data   = Input(UInt(coreDataBits.W))
    val req_coh    = Input(new ClientMetadata)

    val prefetch   = Decoupled(new BoomDCacheReq)
  })
}

/**
  * Does not prefetch
  */
class NullPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
{
  io.prefetch.valid := false.B
  io.prefetch.bits  := DontCare
}

/**
  * Next line prefetcher. Grabs the next line on a cache miss
  */
class NLPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
{

  val req_valid = RegInit(false.B)
  val req_paddr = Reg(UInt(coreMaxAddrBits.W))
  val req_vaddr = Reg(UInt(coreMaxAddrBits.W))  
  val req_cmd   = Reg(UInt(M_SZ.W))

  val mshr_req_paddr = io.req_paddr + cacheBlockBytes.U
  val cacheable = edge.manager.supportsAcquireBSafe(mshr_req_paddr, lgCacheBlockBytes.U)
  when (io.req_val && cacheable) {
    req_valid := true.B
    req_paddr  := mshr_req_paddr
    req_vaddr  := io.req_vaddr
    req_cmd   := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)
  } .elsewhen (io.prefetch.fire) {
    req_valid := false.B
  }

  io.prefetch.valid            := req_valid && io.mshr_avail
  io.prefetch.bits.paddr       := req_paddr
  io.prefetch.bits.vaddr       := DontCare
  io.prefetch.bits.uop         := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := req_cmd
  io.prefetch.bits.data        := DontCare
  io.prefetch.bits.is_hella    := false.B
}

/**
  * Stride prefetcher. Prefetches based on the last stride and current stride
  * @param sbSize Size of the stride buffer 
  */
class StridePrefetcher(implicit edge: TLEdgeOut, p: Parameters, sbSize: Int = 4) extends DataPrefetcher
{
  // parameter checks
  require(isPow2(sbSize), s"Parameter sbSize must be a power of 2, but got $sbSize")
  require(sbSize > 0, s"Parameter sbSize must be greater than 0, but got $sbSize")
  require(sbSize <= 16, s"Parameter sbSize must be less than or equal to 16, but got $sbSize")

  val req_valid = RegInit(false.B)
  val req_paddr = Reg(UInt(coreMaxAddrBits.W))
  val req_vaddr = Reg(UInt(coreMaxAddrBits.W))
  val req_cmd   = Reg(UInt(M_SZ.W))

  // Prefetcher state
  val paddr_buffer  = RegInit(VecInit(Seq.fill(sbSize)(0.U(coreMaxAddrBits.W))))
  val stride_buffer = Reg(Vec(sbSize, SInt(16.W)))
  val valid_stride  = RegInit(VecInit(Seq.fill(sbSize)(false.B)))

  // calculate prefetch addresses and strides for all possible last addresses
  // todo should the number of adders be optimized? maybe assume stride is only a few bytes
  val mshr_req_paddr = VecInit(Seq.tabulate(sbSize)(i => ((io.req_paddr.asSInt + stride_buffer(i)).asUInt)))
  val mshr_req_stride = VecInit(Seq.tabulate(sbSize)(i => (io.req_paddr - paddr_buffer(i)).asSInt))

  // check if the current stride matches any valid strides in the buffer
  // set msb of hit_sel to 1 if there are no matches 
  val hit_seq = (0 until sbSize).map {i => (valid_stride(i) && (mshr_req_stride(i) === stride_buffer(i))) -> i.U}
  val hit_sel = PriorityMux(hit_seq :+ (true.B, (sbSize + 1).U))

  // check if there are any free slots in the stride buffer
  // set msb of miss_sel to 1 if there are no free slots
  val miss_seq = (0 until sbSize).map {i => (~valid_stride(i)) -> i.U}
  val miss_sel = PriorityMux(miss_seq :+ (true.B, (sbSize + 1).U))

  // create random select in case of stride conflict
  val rand_sel = Cat(Seq.tabulate(log2Ceil(sbSize))(i => io.req_paddr(4+i) ^ io.req_paddr(8+i))).asUInt 
  // val rand_sel = Cat(io.req_paddr(8) ^ io.req_paddr(4), io.req_paddr(9) ^ io.req_paddr(5))

  val cacheable = VecInit(Seq.tabulate(sbSize)(i => edge.manager.supportsAcquireBSafe(mshr_req_paddr(i), lgCacheBlockBytes.U)))

  when (io.req_val) {
    when (~hit_sel(hit_sel.getWidth - 1)) { // hit
      req_valid := cacheable(hit_sel)
      req_paddr := mshr_req_paddr(hit_sel)
      req_cmd   := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)
      paddr_buffer(hit_sel) := io.req_paddr
    } .elsewhen (~miss_sel(miss_sel.getWidth - 1)) { // miss with free slot
      stride_buffer(miss_sel) := mshr_req_stride(miss_sel)
      paddr_buffer(miss_sel) := io.req_paddr
      valid_stride(miss_sel) := true.B
    } .otherwise { // miss with no free slot
      stride_buffer(rand_sel) := mshr_req_stride(rand_sel)
      paddr_buffer(rand_sel) := io.req_paddr
    }
  } .elsewhen (io.prefetch.fire) {
    req_valid := false.B
  }

  io.prefetch.valid            := req_valid && io.mshr_avail
  io.prefetch.bits.paddr       := req_paddr
  io.prefetch.bits.vaddr       := DontCare
  io.prefetch.bits.uop         := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := req_cmd
  io.prefetch.bits.data        := DontCare
  io.prefetch.bits.is_hella    := false.B
}

/**
  * Indirect prefetcher. Determines whether data most recently pulled into the
  * cache is a pointer and, if so, prefetches the data at the pointer's location
  */ 
 class IndirectPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
 {
  val req_valid = RegInit(false.B)
  val req_paddr = Reg(UInt(coreMaxAddrBits.W))
  val req_vaddr = Reg(UInt(coreMaxAddrBits.W))
  val req_data  = Reg(UInt(coreDataBits.W))
  val req_cmd   = Reg(UInt(M_SZ.W))

  val mshr_req_addr = io.req_data // If data is an address, we will prefetch from there
  val cacheable = edge.manager.supportsAcquireBSafe(mshr_req_addr, lgCacheBlockBytes.U)
  
  val M = 8 // Width of Compare Bits section
  val N = 4 // Width of Filter Bits section
  val data_addr_match = RegInit(false.B)

  
  when (io.req_val && cacheable) {
    req_valid := true.B
    req_paddr  := mshr_req_addr
    req_vaddr  := io.req_vaddr
    req_cmd   := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)
  } .elsewhen (io.prefetch.fire) {
    req_valid := false.B
  }

  val addr_compare = req_vaddr(coreMaxAddrBits - 1, coreMaxAddrBits - M)
  val addr_filter  = req_vaddr(coreMaxAddrBits - M - 1, coreMaxAddrBits - M - N)
  val data_compare = req_data(coreMaxAddrBits - 1, coreMaxAddrBits - M)
  val data_filter  = req_data(coreMaxAddrBits - M - 1, coreMaxAddrBits - M - N)
  if (addr_compare == data_compare) { // Base address matches
    if ((data_compare == (1 << (N - 1))) && (data_filter != (1 << (N - 1)))) {
      // Base address is all 1's, so check the next M bits for a 0
      data_addr_match := true.B
    } else if ((data_compare == 0.U) && (data_filter != 0.U)) {
      // Base address is all 0's, so check the next M bits for a 1
      data_addr_match := true.B
    }
  } else {
    data_addr_match := false.B
  }

  io.prefetch.valid            := req_valid && io.mshr_avail && data_addr_match
  io.prefetch.bits.paddr       := req_paddr
  io.prefetch.bits.vaddr       := DontCare
  io.prefetch.bits.uop         := NullMicroOp
  io.prefetch.bits.uop.mem_cmd := req_cmd
  io.prefetch.bits.data        := DontCare
  io.prefetch.bits.is_hella    := false.B
}