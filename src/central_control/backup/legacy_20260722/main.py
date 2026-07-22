"""전체 프로젝트 실행 진입점."""

try:
    # Installed ROS 2 package execution.
    from central_control.pipeline import SmartParkingPipeline
except ModuleNotFoundError:
    # Existing source-tree execution: python3 -m src.central_control.main
    from src.central_control.pipeline import SmartParkingPipeline


def main() -> None:
    pipeline = SmartParkingPipeline()
    pipeline.run()


if __name__ == "__main__":
    main()
