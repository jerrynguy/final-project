"""
Query Parser Module
Extracts robot control parameters and stream URLs from natural language queries.
"""

import re
import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)

# Query Parsing Utilities
class QueryExtractor:
    """Parser for extracting robot control parameters from natural language queries."""
    
    def __init__(self):
        """Initialize the query parser."""
        pass

    def extract_stream_url_from_query(self, query: str) -> str:
        """
        Extract RTSP/HTTP stream URL from natural language query.
        """
        logger.info(f"Extracting stream URL from query: '{query}'")
        
        patterns = [
            r"rtsp://[^\s]+",
            r"http://[^\s]+",
            r"(?:stream|camera|video).*?(?:url|link|address)[:\s]+([^\s]+)",
            r"(?:url|link|address)[:\s]+([^\s]+)",
        ]
        
        for pattern in patterns:
            match = re.search(pattern, query, re.IGNORECASE)
            if match:
                found_url = (
                    match.group(0)
                    if "rtsp://" in match.group(0) or "http://" in match.group(0)
                    else match.group(1)
                )
                logger.info(f"Found stream URL: '{found_url}'")
                return found_url.strip()
        
        return ""
    
    def parse_robot_command_from_query(self, query: str) -> Dict[str, str]:
        """
        Parse robot control parameters from natural language query.
        """
        query_lower = query.lower()
        
        # Define keyword mappings for each parameter
        param_keywords = {
            "control_mode": {
                "autonomous": ["autonomous", "auto", "independent"],
            },
            "navigation_goal": {
                "explore": ["explore", "wander", "discover"],
            },
            "safety_level": {
                "high": ["safe", "careful", "slow", "conservative"],
                "low": ["fast", "quick", "aggressive", "bold"],
                "medium": [],
            },
        }
        
        # Default parameters
        params = {
            "control_mode": "autonomous",
            "navigation_goal": "explore",
            "safety_level": "medium",
        }
        
        # Match keywords to set parameters
        for param_name, options in param_keywords.items():
            for value, keywords in options.items():
                if any(word in query_lower for word in keywords):
                    params[param_name] = value
                    break
                    
        return params
    
    def debugging_robot_config_resolution(
        self,
        config,
        builder,
        logger
    ) -> Dict[str, Any]:
        """
        Resolve robot configuration from query and config object.
        """
        try:
            query = builder._workflow_builder.general_config.front_end.input_query[0]
            stream_url = self.extract_stream_url_from_query(query)

            if not stream_url:
                stream_url = config.stream_url

            robot_params = self.parse_robot_command_from_query(query)

            return {
                "stream_url": stream_url,
                "control_mode": robot_params.get("control_mode", config.control_mode),
                "navigation_goal": robot_params.get(
                    "navigation_goal", config.navigation_goal
                ),
                "safety_level": robot_params.get("safety_level", config.safety_level),
                "max_speed": config.max_speed,
                "exploration_speed_boost": getattr(config, 'exploration_speed_boost', 1.0),
            }
        except Exception as e:
            logger.info(f"Could not extract from input_query: {e}")

        # Fallback to config defaults  
        return {
            "stream_url": config.stream_url,
            "control_mode": config.control_mode,
            "navigation_goal": config.navigation_goal,
            "safety_level": config.safety_level,
            "max_speed": config.max_speed,
            "exploration_speed_boost": getattr(config, 'exploration_speed_boost', 1.0),
        }