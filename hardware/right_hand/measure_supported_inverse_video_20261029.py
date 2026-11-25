"""Reuse saved-image geometry analyses for the supported Cube trial."""
from pathlib import Path
import trace_adaptive_loaded_grid_20261029 as grid
import measure_inverse_side_marker_20261029 as side
ROOT=Path(__file__).resolve().parent
F=ROOT/'supported_inverse_trial_20261029_231345_341471';O=F/'offline_review';O.mkdir(exist_ok=True)
grid.F=F;grid.OUT=O;grid.main()
side.F=F;side.O=O;side.main()
