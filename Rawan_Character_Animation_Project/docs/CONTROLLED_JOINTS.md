# Controlled joints

The model controls the 10 required Nathan human joints:

1. leftAnkle
2. rightAnkle
3. leftKnee
4. rightKnee
5. leftHip
6. rightHip
7. leftShoulder
8. rightShoulder
9. leftElbow
10. rightElbow

The visual bridge maps the controller states to available N8RO/Nathan animation codes:

- `Idle Walk Forward` -> WALK-LIKE
- `Idle Shake` -> PUSH-LIKE
- `Idle Breathing` -> CLIMB-LIKE
- `Idle Neutral` -> neutral/rest
