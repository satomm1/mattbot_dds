# Beliefs of all active ledger objects, sorted by belief (least certain first).
std_msgs/Header header
float64 hold_s         # t1
float64 decay_s        # t2
ObjectBelief[] objects
