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
    val req_pc_lob = Input(UInt(log2Ceil(icBlockBytes).W))
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
  
  // ! Debug prints
  if (true) {
    val cycles = RegInit(0.U(32.W))
    cycles := cycles + 1.U

    when (io.prefetch.fire) {
      printf(p"@ [PFETCH] CYCLE: ${cycles} PREFETCHING req_addr 0x${Hexadecimal(req_paddr)}\n")
    }

    when (io.req_val) {
      printf(p"~ [PFETCH] CYCLE: ${cycles} io.req_addr=0x${Hexadecimal(io.req_paddr)} [${Hexadecimal(mshr_req_paddr)}]\n")
    }
  }
}

/**
  * Stride prefetcher. Prefetches based on the last stride and current stride
  * @param sbDepth Size of the stride buffer 
  */
class StridePrefetcher(implicit edge: TLEdgeOut, p: Parameters, sbDepth: Int = 32, sbWidth: Int = 16) extends DataPrefetcher
{
  // parameter checks
  require(isPow2(sbDepth), s"Parameter sbDepth must be a power of 2, but got $sbDepth")
  require(sbDepth > 0, s"Parameter sbDepth must be greater than 0, but got $sbDepth")
  require(sbDepth <= 64, s"Parameter sbDepth must be less than or equal to 64, but got $sbDepth")

  val req_valid = RegInit(false.B)
  val req_paddr = Reg(UInt(coreMaxAddrBits.W))
  val req_cmd   = Reg(UInt(M_SZ.W))

  // Prefetcher state
  val paddr_lob_buffer = Reg(Vec(sbDepth, UInt(sbWidth.W)))
  val stride_buffer    = Reg(Vec(sbDepth, SInt(sbWidth.W)))
  val valid_buffer     = RegInit(VecInit(Seq.fill(sbDepth)(false.B)))

  // index using pc low order bits
  val idx = io.req_pc_lob(5,1)
  

  val mshr_req_paddr = Wire(UInt(coreMaxAddrBits.W))
  mshr_req_paddr := ((io.req_paddr).asSInt + stride_buffer(idx)).asUInt

  val mshr_req_stride = Wire(SInt(16.W))
  mshr_req_stride := (io.req_paddr(sbWidth-1, 0) - paddr_lob_buffer(idx)).asSInt

  val cacheable = edge.manager.supportsAcquireBSafe(mshr_req_paddr, lgCacheBlockBytes.U)

  when (io.req_val) {
    when(valid_buffer(idx)) {
      when (mshr_req_stride === stride_buffer(idx) && mshr_req_stride =/= 0.S && cacheable) {
        req_valid := true.B
        req_paddr := mshr_req_paddr
        req_cmd   := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)
      } .otherwise {
        stride_buffer(idx) := mshr_req_stride
      }
    } .otherwise {
      valid_buffer(idx) := true.B
      stride_buffer(idx) := cacheBlockBytes.S
    }
    paddr_lob_buffer(idx) := io.req_paddr(sbWidth-1, 0)
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

  // ! Debug prints
  if (true) {
    val cycles = RegInit(0.U(32.W))
    cycles := cycles + 1.U

    when (io.prefetch.fire) {
      // printf(p"@ [PFETCH] CYCLE: ${cycles} PREFETCHING req_addr 0x${Hexadecimal(req_paddr)}\n")
    }

    when (io.req_val) {
      // printf(p"~ [PFETCH] CYCLE: ${cycles} io.req_addr=0x${Hexadecimal(io.req_paddr)} [${Hexadecimal(mshr_req_paddr)}] lob=${io.req_pc_lob(0)} idx=${idx}")
      // printf(p"[${mshr_req_stride} - ${stride_buffer(idx)}] [${Hexadecimal(io.req_paddr(sbWidth-1, 0))} - ${Hexadecimal(paddr_lob_buffer(idx))}]\n")
    }
  }
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