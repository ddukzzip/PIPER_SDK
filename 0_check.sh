# Listen-only diagnostics: feedback alive? leader command frames visible? follow lag?
# Move the leader arm by hand while this runs.
python examples/piper/piper_check.py monitor --port ${1:-can0} --seconds 10
