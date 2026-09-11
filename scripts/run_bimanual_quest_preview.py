"""Compatibility entry point for the hardware-free bimanual Quest preview."""
import sys
from retargeting_apps.bimanual_quest import main

if __name__ == "__main__":
    if any(arg == "--backend" or arg.startswith("--backend=") for arg in sys.argv[1:]):
        raise SystemExit("Use python -m retargeting_apps.bimanual_quest for backend selection")
    main()
