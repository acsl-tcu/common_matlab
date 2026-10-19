function af7r58m_capture_timing_ring(obj,t_hard_cycle)
% AF7R58M bounded diagnostic recorder.  This function must never throw.
tcap=tic;
try
    if ~(isa(obj,'REPLANNING_SYSTEM')&&logical(obj.ENABLE_AF7R58M_TIMING_RING))
        return;
    end
    cap=double(obj.af7r58m_timing_ring_capacity);
    wid=double(obj.af7r58m_timing_ring_width);
    if ~(isscalar(cap)&&isscalar(wid)&&isfinite(cap)&&isfinite(wid)&&cap==64&&wid==32)
        return;
    end
    if ~isequal(size(obj.af7r58m_timing_ring),[64 32])
        return;
    end

    r=nan(1,32);
    r(1)=double(obj.generator_request_count);
    r(2)=1e3*toc(t_hard_cycle);
    r(3)=double(obj.generator_last_cycle_critical_ms);
    r(4)=double(obj.generator_last_elapsed_before_ms);
    r(5)=double(obj.generator_last_budget_remaining_ms);
    r(6)=double(obj.generator_last_context_ms);
    r(7)=double(obj.generator_last_callback_ms);
    r(8)=double(obj.generator_last_validation_ms);
    r(9)=double(obj.generator_last_critical_ms);
    r(10)=double(obj.generator_last_runtime_ms);
    r(11)=double(obj.generator_last_status_code);
    r(12)=double(obj.generator_deadline_fail_count);
    r(13)=double(obj.generator_candidate_count);
    r(14)=double(obj.generator_validation_pass_count);
    r(15)=double(obj.generator_validation_fail_count);
    r(16)=double(obj.active_reference_version);
    r(17)=double(obj.replan_commit_count);
    r(18)=double(obj.reference_only_evaluation_count);
    r(19)=double(obj.reference_only_commit_count);
    r(20)=double(obj.reference_only_runtime_packet_reuse_count);
    r(21)=double(obj.reference_only_runtime_packet_rebuild_count);
    r(22)=double(obj.reference_only_fast_packet_rebuild_count);
    r(23)=double(obj.compact_native_proposal_count);
    r(24)=double(obj.r9_stage_episode_ms);
    r(25)=double(obj.r9_synth_hessian_ms);
    r(26)=double(obj.r9_synth_corridor_ms);
    r(27)=double(obj.r9_synth_qp_ms);
    r(28)=double(obj.r9_synth_candidate_build_ms);
    r(29)=double(obj.r9_stage_synthesis_ms);
    r(30)=double(obj.r9_stage_candidate_loop_ms);
    r(31)=double(obj.r9_stage_total_internal_ms);
    r(32)=double(obj.r9_hr_callback_wall_ms);

    head=double(obj.af7r58m_timing_ring_head);
    idx=mod(head,cap)+1;
    obj.af7r58m_timing_ring(idx,:)=r;
    obj.af7r58m_timing_ring_head=idx;
    obj.af7r58m_timing_ring_count=min(cap,double(obj.af7r58m_timing_ring_count)+1);

    us=1e6*toc(tcap);
    obj.af7r58m_last_capture_overhead_us=us;
    if isfinite(us)
        obj.af7r58m_max_capture_overhead_us=max(double(obj.af7r58m_max_capture_overhead_us),us);
    end
catch
    % Diagnostic instrumentation must never alter control flow.
end
end
