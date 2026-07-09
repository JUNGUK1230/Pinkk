"""전체 프로젝트 실행 진입점."""

from src.central_control.pipeline import SmartParkingPipeline


def main() -> None:
    pipeline = SmartParkingPipeline()
    pipeline.run()


if __name__ == "__main__":
    main()
