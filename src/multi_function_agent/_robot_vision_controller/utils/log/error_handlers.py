"""
Error Handlers Module
Centralized error response generators for robot vision controller.
"""

from typing import Callable, Awaitable


class ErrorHandlers:
    """Reusable async error response generators."""
    
    @staticmethod
    def no_stream_url() -> Callable[[str], Awaitable[str]]:
        """Generate error response for missing stream URL."""
        async def error_fn(dummy: str) -> str:
            return """ERROR: No stream URL provided
Please specify the robot camera stream, for example:
• "Control robot using rtsp://127.0.0.1:8554/robotcam"  

The robot controller needs a valid video stream to perform vision-based navigation."""
        return error_fn
    
    @staticmethod
    def invalid_stream(stream_url: str, error_msg: str) -> Callable[[str], Awaitable[str]]:
        """Generate error response for invalid stream connection."""
        async def error_fn(dummy: str) -> str:
            return f"""STREAM CONNECTION ERROR: {error_msg}
Please ensure stream URL is correct: {stream_url}"""
        return error_fn
    
    @staticmethod
    def control_error(error: Exception, stream_url: str, model_ready: bool) -> Callable[[str], Awaitable[str]]:
        """Generate error response for control loop failure."""
        model_status = "READY" if model_ready else "NOT LOADED"
        
        async def error_fn(dummy: str) -> str:
            return f"""ROBOT CONTROL ERROR: Could not complete vision-based control

Error Details: {str(error)}

System Status:
• Model Status: {model_status}
• Stream URL: {stream_url}

Please check the error details and ensure all components are functioning correctly."""
        return error_fn