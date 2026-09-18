# YO_ERROR-404 Future Engineers Testing

This repository is the shared workspace for testing, collaboration and evidence for the YO_ERROR-404 WRO Future Engineers robot.

## What belongs here

| Folder | Use it for |
| --- | --- |
| `docs/test-plans/` | Planned tests, course setup and acceptance criteria |
| `docs/calibration/` | Steering, encoder, sensor and camera calibration records |
| `run-logs/` | Results from each practice run and the next corrective action |
| `firmware/` | Tested firmware snapshots and release notes |
| `media/` | Labelled photos and short videos that support a test result |

## Collaboration routine

1. Create an issue for each bug, improvement or test request.
2. Use a branch for the work and describe the test performed in the pull request.
3. Add the result to `run-logs/` after every meaningful course run.
4. Record a calibration change before using it in a competition run.
5. Do not commit passwords, tokens, Wi-Fi credentials or other secrets.

## Test record format

Each log should state the date, vehicle, firmware revision, course condition, expected behaviour, actual result, failure reason and next action. Use the templates in this repository to keep reports comparable.
