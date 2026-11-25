# 최초 거리 질의 후보는 채택하지 않음

This folder preserves an exploratory run that trusted `mj_geomDistance == 0`.
The later `after_transfer/distance_query_audit.json` shows zero distances with
witness points separated by many millimeters and no actual contact. Therefore
the 22 initial `geometry_accepted` flags do **not** establish ring contact and
must not be used to approve a path. All three exploratory dynamic continuations
failed the full grip task and no hardware command was provided.

The corrected search is in `../ring_third_support_v2/after_transfer/review.json`.
It requires an actual pinned-pose contact before geometric admission and rejects
zero queries with separated witness points. This changes only research proposals,
not the original model, collision flags, limits, or runtime.
