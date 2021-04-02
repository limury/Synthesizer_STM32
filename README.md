
---
noteId: "b5acef40864f11ebbe1f35d2d4b4ff54"
tags: []

---

# Embedded_Synth

## NOTE: FINAL CODE IS ON THE FINAL BRANCH, NOT ON MAIN
## How we are going to do polyphony and complex sounds

- We are updating the voltage in the speaker at 22000 Hz.
- We are trying to recrease a certain frequency.

Approach:
- To create a wave at 440 Hz we need to iterate over a sine wave 440 times per second. 
- If we get a single reading from an array into the speaker. This means we iterate over 22000 items of an array in 1 second. Hence, for this to be equal to 440 waves per second we need an array of 50 items (22000 / 440).
- Another way to accomplish the same result is that we have a larger array (say 200 items) and we read 1 in each 4 values (meaning we still produce 440 waves per second).
- The way we implement this is that we simply have an array with a sine wave in it, and the pointer increase at each timestep is dependant on the frequency of the note.
- So if we're using an array of size 200 (which stores a single sine wave), to get 440 Hz output, the increase would be of exactly 4. Hence at each loop we write `it += 4`.
- For a sightly higher frequency we simply make the increase a bit larger (`it += 4.xx`). 
- Hence to reproduce multiple frequencies at the same time, lets say 2 notes. The way we do it is: `note_1_it += x`, `note_2_it += y`. \ Then: `output_voltage = SAMPLE_ARRAY[ note_1_it ] + SAMPLE_ARRAY[ note_2_it ]`.
