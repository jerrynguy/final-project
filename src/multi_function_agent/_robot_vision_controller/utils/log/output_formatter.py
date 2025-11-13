"""
Output Formatter Module
Format control loop results into human-readable output.
"""

from typing import Dict, Any


class OutputFormatter:
    """Format robot control results for display."""
    
    @staticmethod
    def format_control_results(
        results: Dict[str, Any],
        stream_url: str,
        mode: str,
        mission = None
    ) -> str:
        """
        Format control loop results into human-readable output.
        """
        output = f"""ROBOT VISION CONTROL RESULTS
{'='*50}

STREAM SOURCE: {stream_url}
MODE: {mode}
ITERATIONS: {results['iterations']}
FINAL STATUS: {results['final_status']}
"""
        
        # Add mission information if available
        if mission:
            output += f"\nMISSION: {mission.description}\n"
            output += f"TYPE: {mission.type}\n"
            if hasattr(mission, 'target_class') and mission.target_class:
                output += f"TARGET: {mission.target_class}\n"
        
        # Add command history
        output += "\nCOMMANDS EXECUTED (last 10):\n"
        for i, cmd in enumerate(results["commands_sent"][-10:], 1):
            output += f"{i}. {cmd.get('action', 'unknown')}: {cmd.get('parameters', {})}\n"
        
        # Add obstacle statistics
        unique_obstacles = len(set(str(obs) for obs in results["obstacles_detected"]))
        output += f"\nOBSTACLES ENCOUNTERED: {unique_obstacles}"
        
        # Add navigation statistics
        successful_decisions = sum(
            1 for d in results["navigation_decisions"] if d.get("success")
        )
        total_decisions = len(results["navigation_decisions"])
        output += f"\nSUCCESSFUL NAVIGATION DECISIONS: {successful_decisions}/{total_decisions}"
        
        return output
    
    @staticmethod
    def format_stream_stats(stream_stats: Dict[str, Any]) -> str:
        """
        Format streaming performance statistics.
        """
        stream_info = stream_stats.get('stream_info', {})
        
        output = f"""STREAMING PERFORMANCE:
• Total frames processed: {stream_stats.get('total_frames', 0)}
• Frame drop rate: {stream_stats.get('drop_rate_percent', 0):.1f}%
• Stream resolution: {stream_info.get('width', 0)}x{stream_info.get('height', 0)}"""
        
        return output