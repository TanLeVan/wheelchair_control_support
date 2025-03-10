## 14/12/2024
Begin implementing gap finding algorithm from Potential Gap paper.

## 14/1/2025
class Gap:
- frame
- left_idx
- ldist (distant to left point)
- right_idx
- rdist

- isAxial()
{
    verify that the gap is axial from the left_idx, right_idx. ldist, rdist and resolution of the scan
}


Algorithm for to find gap:
- clear last observed gap set
- store_scan_msg = scan message
- half_scan = half of scan message size
- max_scan_dist = maximum distance in the scan
- min_dist = minimum distance in the scan
- gap_size = 0
- gap_ldist = first reading of the scan
- last_scan = first reading of the scan
- prev_lgap = True if gap_ldist >= max_scan_dist; False otherwise. # This variable is True when the system is alreading tracking a gap
                                                                  # false when no gap is tracked

- For range in scan range:
    - scan_dist = range
    - scan_diff = scan_dist - last_scan (different between current scan and last scan)
    - if (abs(scan_diff)> threshold)
        - if (scan_dist < max_scan_dist && last_scan < max_scan_dist) #If current scan value and last scan value are not infinity
            - Initialize a detected_gap with left_idx = last scan id, gap_ldist = last_scan
            - Add right information to the detected_gap with right_idx = scan_dist id, gap_rdist = scan_dist
            - set minimum safe distance?
            - if gap distance > 2*robot radius => push detected_gap to observed_gaps
    
    - if one of the two value (last_scan or scan_dist) is infinity and the other is less than infinity (detection of swept gap)
        - if prev_lgap # meaning the system is tracking a gap. This means the current scan point is the end of a gap
            - prev_lgap = false
            - create a new detected_gap with left information
            - Add right information (current scan point)
            - push gap to observed_gap if gap length > 2*robot radius
        - If not tracking a gap, the marking the left side of a new gap:
            - gap_lidx = it - 1;
            - gap_ldist = last_scan;
            - prev_lgap = true;

    // After processing the current reading, the current reading is assigned to last reading
    - last_scan = scan_dist
    
// After looping throught the scan, if prev_lgap = true, that mean a gap has not been close => Closing the last gap
// If a left side of a gap is found near the end of the scan and the right side is near the 
// begining, its need to be handled properly
- if prev_lgap  
    - Add left gap information as current scan information
    - Right gap information is the last scan point
    - Add gap to observed_gap if gap distance > 2* robot radius
// Brigde last gap and first gap incase first reading is infinity
- If observed_gaps.size is 2 or more
    - If first gap left id is 0 and last gap right id is last scan
        // In this situation, the Left distance of first gap and the 
        right distance of second gap need to be adjusted for scan consitancy

// Note: left gap means along the scan direction, the distance to the first edge
// is smaller than distance to second edge



Algorithm to merge gap:
second_gap
start = true
- for gap in observed_gap:
    - if (start and gap is axial and gap is left type)
        // axial left type gap mark the begining of a mergable gap
        - start = false
        - push gap to second_gaps
    - else
        if start: // start gap but radial or right type
            // These gap will not be merged
            - push to second_gap.
        if not start:
            - if gap is axial
                - if gap is left type  
                    push gap to second_gap
                - else:
                    // If right gap type
                    // Merge with previous left gap type
                    - current_rdist = right distance of gap
                    - erase_counter = 0
                    - last_mergable = -1
                    - For second_gap in second_gaps in backward order:
                        // find start_idx , which is gap right index
                        // find end_idx, which is second_gap left index
                        // Find the smallest range scan from start_idx to end_idx
                        // first_cond = Test if smallest_range_scan <= start_idx range and <= end_idx range
                        // second_cond = If second_gap is left type or not axial 
                        // third_cond = if right index of gap - left index of second_gap < MAX_IDX_DIFF
                        if first_cond and second_cond and third_cond:
                            second_gap is last_mergable
                    - if (last_mergable!=-1)
                        - erase gap with position from last_mergable+1 to last in second_gap
                        - Add right information of the last gap in the second_gap
                        // second_gap.back().addRightInformation(observed_gaps[i].RIdx(), observed_gaps[i].RDist());
                    - else:
                        push gap to second_gaps
            - else // if gap is not axial (radial gap)
                // Merge to last gap in second_gap
                - current_dist = right distance of gap
                - if current_dist - left dist of last element in second_gap < 