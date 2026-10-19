AF7R57N V7 — exception-free episode resource return
======================================================

V6 closed the resource guard itself. V7 normalizes only resource reasons 202/203 from the pre-existing fixed-episode exception into ordinary PlanResult rejection using the same nearby result_struct serializer and context arguments. Non-resource fixed-episode failures still throw.

V7 requires all exercised resource faults to use RETURN transport, plus 2000 current admitted QPC trials and 1000 trials each for 202 and 203 ordinary rejection, with every measured trial <4 ms. Production cap values are still not selected.
