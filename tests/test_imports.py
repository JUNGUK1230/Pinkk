def test_project_imports() -> None:
    from src.system_manager.pipeline import SmartParkingPipeline

    assert SmartParkingPipeline is not None
