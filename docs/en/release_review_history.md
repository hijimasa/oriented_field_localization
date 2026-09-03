# Release review history

English | [日本語](../release_review_history.md)

This page summarizes the current public-release decision. See [`../reviews/`](../reviews/) for
the findings and responses.

| ID | Date | Target | Findings | Response | Decision |
|---|---|---|---|---|---|
| R01 | 2026-09-03 | `698f134` + R01 response commit | [Critical 0 / High 3 / Medium 9 / Low 5](../reviews/r01-2026-09-03-findings.md) | [All items addressed by implementation or release policy](../reviews/r01-2026-09-03-response.md) | **Go** |

## Current state

All three High, nine Medium, and five Low findings have been addressed. The ROS 2 Humble build,
five tests, installed downstream linking, minimal evaluation, positive/negative public snapshot
checks, and a 10-second Gazebo smoke run pass. The release owner accepts the old text as development
history referring to a sibling project planned for later publication, so the existing Git history
may be retained. The one-commit snapshot is optional rather than a release condition. Long-running
Nav2 simulation and a real ROS integration test with nonzero LiDAR extrinsics remain pre-deployment
validation items.
