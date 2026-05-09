def should_force_material_refresh(
    *,
    now_sec: float,
    initial_sync_complete: bool,
    force_refresh_until_sec: float,
    last_full_refresh_sec: float,
    steady_refresh_sec: float,
) -> bool:
    if not initial_sync_complete:
        return True
    if now_sec < force_refresh_until_sec:
        return True
    return steady_refresh_sec > 0.0 and now_sec - last_full_refresh_sec >= steady_refresh_sec
