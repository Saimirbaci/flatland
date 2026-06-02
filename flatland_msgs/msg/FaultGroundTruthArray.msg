# Sealed out-of-band ground-truth snapshot of all configured faults at one
# instant. Published ONLY on the reserved /_ground_truth namespace; excluded
# from the RCA bag. See FaultGroundTruth.msg.
Header header
FaultGroundTruth[] faults
