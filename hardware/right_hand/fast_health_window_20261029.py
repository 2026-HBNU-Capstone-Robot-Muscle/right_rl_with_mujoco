"""Integration metadata; fast decoder/transport retains its tested source hash."""
WINDOW_POLICY=dict(version='fast_health8_and_original_OFF16_QPC_v1_20261029',budget_clock='time.perf_counter',
    packet_window_ms=40.,torque_off_packet_window_ms=80.,scopes=[[64,83],[64,1]],
    overall_health_sample_budget_ms=200,two_fresh_healthy_snapshots_after_recovery=True,
    goal_or_setting_write_retries=0,missing_or_corrupt_data_accepted=False,hardware_health_limits_unchanged=True,
    per_packet_window_capped_by_sample_deadline=True,full_health_group_size=8,health_read_instruction=0x8a,
    torque_off_readback_group_size=16,all16_fresh_before_return=True,reused_half_snapshots_after_failure=False)
