package boom

import Chisel._
import freechips.rocketchip.config.{Parameters, Field}

case class Lab3Parameters(
  enabled: Boolean = true,
  num_entries_local: Int = 128,
  history_length_local: Int = 5,
  history_length: Int = 10,
  info_size: Int = 0
  )

case object Lab3Key extends Field[Lab3Parameters]

class TournamentResp(gindex_sz: Int, lindex_sz: Int, lhist_sz: Int) extends Bundle
{
   val gindex = UInt(width = gindex_sz) 
   val lindex = UInt(width = lindex_sz) 
   val lhist  = UInt(width = lhist_sz) 
   val gtaken = UInt(width = 1)
   val ltaken = UInt(width = 1)

   override def cloneType: this.type = new TournamentResp(
      gindex_sz,lindex_sz,lhist_sz).asInstanceOf[this.type]
}

class Lab3BrPredictor(
    fetch_width: Int,
    num_entries_local: Int = 256,
    history_length_local: Int = 10,
    history_length: Int)(implicit p: Parameters)
      extends BrPredictor(fetch_width, history_length)(p)
{
  // Implementing a Tournament Predictor
  require (fetch_width == 1)

  val num_entries_ghist = 1 << history_length
  val num_entries_lhist = 1 << history_length_local

   private def Hash (addr: UInt, hist: UInt) = (addr >> UInt(log2Up(coreInstBytes))) ^ hist

  // Initiallize the predictor and selector states
  val counter_selector = Reg(
    init = Vec(Seq.fill(num_entries_local) {UInt("b00",width = 2)}))
  // Two-level adaptive local predictor
  val table_lhist = Reg(
    init = Vec(Seq.fill(num_entries_local) {UInt("b00000",width = history_length_local)}))
  val counter_lhist = Reg(
    init = Vec(Seq.fill(num_entries_lhist) {UInt("b000",width = 3)}))
  // 2-bit global predictor
  val counter_ghist = Reg(
    init = Vec(Seq.fill(num_entries_ghist) {UInt("b00",width = 2)}))

   println("\t A ghist counter table with " + num_entries_ghist + " entries is constructed")

  // -----------------------------------
  // Stage 1:
  // index into the table to get the count 
  val stall = !io.resp.ready

  // index to the local history table
  val s1_pc = io.req_pc
  val s1_ridx_l = s1_pc >> UInt(log2Ceil(coreInstBytes))
  val s1_ridx_lhist = table_lhist(s1_ridx_l)
  val s1_ridx_ghist = Hash(s1_pc, this.ghistory)

  // gshare idx is also used for arbiter
  val s1_predict_lhist = counter_lhist(s1_ridx_lhist)(2)
  val s1_predict_ghist = counter_ghist(s1_ridx_ghist)(1)
  val s1_select        = counter_selector(s1_ridx_ghist)(1)

  // -------------------------------------
  // S1/S2 Pipeline Reg:
  // Record prediction and info from S1
  val resp_info = Wire(new TournamentResp(
                    history_length,log2Up(num_entries_local),history_length_local))
  resp_info.gindex := RegNext(s1_ridx_ghist)
  resp_info.lindex := RegNext(s1_ridx_l)
  resp_info.lhist  := RegNext(s1_ridx_lhist)
  resp_info.gtaken := RegNext(s1_predict_ghist)
  resp_info.ltaken := RegNext(s1_predict_lhist)
  // if select === 1, send prediction of ghist counter
  // else, select prediction of local counter
  val s2_predict = RegEnable(Mux(s1_select,
                                 s1_predict_ghist,s1_predict_lhist),
                                 !stall)

  // ----------------------------------------------------
  // Stage 2: Send the predictions and info from S1
  io.resp.valid       := !this.disable_bpd
  io.resp.bits.takens := s2_predict
  // Record the index for commit
  io.resp.bits.info   := resp_info.asUInt


  // -----------------------------------------------------
  // Check outcome on Commit. 
  private def CounterUpdate2b (incr: Bool, count: UInt) = Mux(
    incr,
    Mux(count === "b11".U, count, count + 1.U),
    Mux(count === "b00".U, count, count - 1.U))

  private def CounterUpdate3b (incr: Bool, count: UInt) = Mux(
    incr,
    Mux(count === "b111".U, count, count + 1.U),
    Mux(count === "b000".U, count, count - 1.U))

  val commit_info = new TournamentResp(
                    history_length,log2Up(num_entries_local),history_length_local).
                    fromBits(this.commit.bits.info.info)
  
  // From the commit resp packet
  // Check mispredicts and counts
  val commit_s1_en = this.commit.valid
  val commit_s1_gidx  = commit_info.gindex
  val commit_s1_lidx  = commit_info.lindex
  val commit_s1_lhist = commit_info.lhist
  val commit_s1_taken = this.commit.bits.ctrl.taken(0)
  val commit_s1_g1different = commit_info.gtaken(0) ^ commit_info.ltaken(0)
  val commit_s1_lmispredict = commit_info.ltaken(0) ^ commit_s1_taken

  // -------------------------------------
  // S1/S2 Pipeline Reg:
  // Record outcome, mispredicts and counts from S1
  val commit_s2_gidx  = RegEnable(commit_s1_gidx, commit_s1_en)
  val commit_s2_lidx  = RegEnable(commit_s1_lidx, commit_s1_en)
  val commit_s2_lhist = RegEnable(commit_s1_lhist, commit_s1_en)
  val commit_s2_gcount = RegEnable(counter_ghist(commit_s1_gidx),
                                  commit_s1_en)
  val commit_s2_lcount = RegEnable(counter_lhist(commit_s1_lhist),
                                  commit_s1_en)

  // gshare idx is also used for arbiter, check if this is better
  val commit_s2_scount = RegEnable(counter_selector(commit_s1_gidx),
                                  commit_s1_en)
  val commit_s2_taken = RegEnable(commit_s1_taken, commit_s1_en)
  val commit_s2_en = RegNext(commit_s1_en)
  
  val commit_s2_gldifferent = RegEnable(commit_s1_g1different,
                                        commit_s1_en)
  val commit_s2_lmispredict = RegEnable(commit_s1_lmispredict,
                                        commit_s1_en)

  // update the local history table using commit_s1_lidx
  table_lhist (commit_s2_lidx) := (table_lhist (commit_s2_lidx) 
                                   << 1) | commit_s2_taken(0)
  // ----------------------------------------------------
  // Stage 2: Update counter table based on commit message
  when (commit_s2_en) {
    // Increment both ghist and local counters if branch is taken
    val commit_s2_gupdate = CounterUpdate2b(commit_s2_taken,commit_s2_gcount)
    val commit_s2_lupdate = CounterUpdate3b(commit_s2_taken,commit_s2_lcount)
    counter_ghist   (commit_s2_gidx) := commit_s2_gupdate 
    counter_lhist  (commit_s2_lhist) := commit_s2_lupdate 

    // Update selector table only if ghist and lhist made different predictions
    when (commit_s2_gldifferent) {

      // Increment counter if local counter mispredicts, 
      // otherwise, decrement counter
      val commit_s2_supdate = CounterUpdate2b(commit_s2_lmispredict,commit_s2_scount)
    // gshare idx is also used for arbiter, check if this is better
      counter_selector (commit_s2_gidx) := commit_s2_supdate } 
   }
}
