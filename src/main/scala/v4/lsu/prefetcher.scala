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
    val req_paddr   = Input(UInt(coreMaxAddrBits.W))
    val req_vaddr   = Input(UInt(coreMaxAddrBits.W))
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
  val req_paddr  = Reg(UInt(coreMaxAddrBits.W))
  val req_vaddr  = Reg(UInt(coreMaxAddrBits.W))  
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
  * Stride prefetcher. Grabs the next line on a cache miss
  */
class StridePrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
{

  val req_valid = RegInit(false.B)
  val req_paddr  = Reg(UInt(coreMaxAddrBits.W))
  val req_vaddr  = Reg(UInt(coreMaxAddrBits.W))  
  val req_cmd   = Reg(UInt(M_SZ.W))

  val last_addr = RegInit(VecInit(Seq.fill(4)(0.U(coreMaxAddrBits.W))))
  val stride = Reg(Vec(4, SInt(16.W)))
  val confidence = RegInit(VecInit(Seq.fill(4)(false.B)))
  
  val hit_sel = Wire(UInt(3.W))
  val miss_sel = Wire(UInt(3.W))

  // create a random two bit in case of stride conflict
  val rand = Cat(io.req_paddr(8) ^ io.req_paddr(4), io.req_paddr(9) ^ io.req_paddr(5))

  // calculate current stride for all possible last addresses
  val curr_stride = VecInit(Seq.tabulate(4)(i => (io.req_paddr - last_addr(i)).asSInt))

  // calculate prefetch address for all possible last addresses
  val mshr_req_paddr = VecInit(Seq.tabulate(4)(i => ((last_addr(i).asSInt + stride(i)).asUInt)))

  when (curr_stride(0) === stride(0)) {
    hit_sel := 0.U
  } .elsewhen (curr_stride(1) === stride(1)) {
    hit_sel := 1.U
  } .elsewhen (curr_stride(2) === stride(2)) {
    hit_sel := 2.U
  } .elsewhen (curr_stride(3) === stride(3)) {
    hit_sel := 3.U
  } .otherwise {
    hit_sel := 5.U
  }

  when (confidence(0)) {
    miss_sel := 0.U
  } .elsewhen (confidence(1)) {
    miss_sel := 1.U
  } .elsewhen (confidence(2)) {
    miss_sel := 2.U
  } .elsewhen (confidence(3)) {
    miss_sel := 3.U
  } .otherwise {
    miss_sel := 5.U
  }

  val cacheable = VecInit(Seq.tabulate(4)(i => edge.manager.supportsAcquireBSafe(mshr_req_paddr(i), lgCacheBlockBytes.U)))

  when (io.req_val) {
    when (~hit_sel(2) && confidence(hit_sel(1, 0))) {
      req_valid := cacheable(hit_sel)
      req_paddr := (io.req_paddr.asSInt + stride(hit_sel)).asUInt
      req_vaddr := io.req_vaddr
      req_cmd   := Mux(ClientStates.hasWritePermission(io.req_coh.state), M_PFW, M_PFR)
      last_addr(hit_sel) := io.req_paddr
    } .elsewhen (~miss_sel(2)) {
      stride(miss_sel) := curr_stride(miss_sel)
      last_addr(miss_sel) := io.req_paddr
      confidence(miss_sel) := 1.U
    } .otherwise {
      stride(rand) := curr_stride(rand)
      last_addr(rand) := io.req_paddr
      confidence(rand) := 1.U
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
  * Indirect prefetcher. Grabs the next line on a cache miss
  */
class IndirectPrefetcher(implicit edge: TLEdgeOut, p: Parameters) extends DataPrefetcher
{

  io.prefetch.valid := false.B
  io.prefetch.bits  := DontCare
}



