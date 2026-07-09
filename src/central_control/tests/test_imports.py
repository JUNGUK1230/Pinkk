def test_project_imports() -> None:
    from src.central_control.pipeline import SmartParkingPipeline

    assert SmartParkingPipeline is not None
