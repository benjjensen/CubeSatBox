## Explanation of each run 

### Trials 
- Trials 1-5: Sun vector was randomly sampled. Didn't have good results 
- Trial 6: Rotation about +X axis, so that `sᴮ = [0, cos(i), sin(i)]`
- Trial 7: Rotation about +Y axis, so that `sᴮ = [cos(i), 0, sin(i)]`
- Trial 8: Rotation about +Z axis, so that `sᴮ = [cos(i), sin(i), 0]` (?)
- Trials 9-10: Random initial state, which was propagated using `rk4`

### Runs 
No particular reason for the name change. These all rely on random initial states and `rk4` for propagation, and all assume that the inertial sun vector is just `[1, 0, 0].`
Due to hardware issues, each run has poritons that are all zero; these are separated out 
into their own segments:

- Run 1: Segment 1: 1-127 (plus some short ones)
- Run 2: Segment 2: 1-170; Segment 3: 221-500 
- Run 3: Segment 4: 1-36 ; Segment 5: 94-312
- Run 4: Segment 6: 73-500 
- Run 5: Segment 7: 1-53 ; Segment 8: 135-473 (Note that this one starts spinning quite quicly)
