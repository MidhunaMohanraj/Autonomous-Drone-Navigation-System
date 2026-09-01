"""
Central configuration. Loads settings from a .env file (see .env.example).
"""
import os
from dotenv import load_dotenv

load_dotenv()


class Config:
    GEMINI_API_KEY: str = os.getenv("GEMINI_API_KEY", "")
    VISION_CHECK_INTERVAL: int = int(os.getenv("VISION_CHECK_INTERVAL", "5"))
    GEMINI_MODEL: str = os.getenv("GEMINI_MODEL", "gemini-2.0-flash")

    @classmethod
    def require_gemini_key(cls) -> str:
        if not cls.GEMINI_API_KEY:
            raise RuntimeError(
                "GEMINI_API_KEY is not set. Copy .env.example to .env and add your "
                "free key from https://aistudio.google.com/apikey"
            )
        return cls.GEMINI_API_KEY
