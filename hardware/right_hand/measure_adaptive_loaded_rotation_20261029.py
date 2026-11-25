"""Saved green-grid/ID5 analysis; no devices and no new acceptance threshold."""
import measure_successful_quarter_20261028 as measure
from pathlib import Path
ROOT=Path(__file__).resolve().parent
measure.FOLDER=ROOT/'adaptive_yaw_loaded_20261029_160451_416563'
measure.OUT=measure.FOLDER/'offline_review';measure.OUT.mkdir(exist_ok=True)
if __name__=='__main__':measure.main()
