# Sealed out-of-band ground-truth label for a single injected fault.
#
# This message is published ONLY on the reserved /_ground_truth namespace and
# written to the sealed sidecar manifest. It is consumed exclusively by the
# offline evaluation harness. It must NEVER be embedded in, or recorded
# alongside, the in-band sensor/odom/tf stream the RCA reasons over.
string fault_id              # unique id from the world YAML
string fault_type            # taxonomy type, e.g. "sensor_bias", "torque_loss"
string affected_model        # model the fault targets
string affected_component    # plugin name the fault targets
string affected_topic        # documentation: the in-band topic perturbed
time onset_time              # sim time the fault became active
time end_time                # sim time the fault ended (zero while still active)
float64 peak_severity        # configured peak severity in [0,1]
float64 current_severity     # severity at the message stamp, in [0,1]
string trigger_description   # human-readable trigger summary
