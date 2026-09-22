# Finer-step knee refinement

Three deterministic 62.5 µs, five-second native probes vary only landing knee
angle relative to full-launch-open candidate 14: +0.002, +0.005, +0.010 rad.
All use immutable producer `0c84bfdec8bdbdd3a4d9724e5bf13469fc895243`, full
contacts, 0.001 Nm command headroom, and unchanged physical audit gates.

All complete five seconds without a fall. Peak speed/rating is 1.04391632,
1.03942852, and 1.03506317 respectively; realized effort, joint position,
flight/rotation and full-contact geometry gates pass. Final-second base speed,
standing error and continuous support still fail during the ten-second recovery
phase. These are screening candidates, not qualified backflips.

Select +0.005 rad by the lowest existing diagnostic loss (4.25335351), not by
relaxing the qualification gates. Its identical candidate/producer now runs
15-second validations at 500, 125 and 62.5 µs. No success GIF yet.
