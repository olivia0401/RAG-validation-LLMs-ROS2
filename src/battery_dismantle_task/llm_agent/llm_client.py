"""
LLM客户端 - 支持多种后端
"""
import os
import asyncio
import aiohttp
from typing import Optional


class LLMClient:
    """统一的LLM客户端接口"""

    def __init__(self, backend: str = "chutes", api_key: Optional[str] = None):
        """
        初始化LLM客户端

        Args:
            backend: "chutes", "ollama", "openai"
            api_key: API密钥(如果需要)
        """
        self.backend = backend
        self.api_key = api_key or os.getenv("CHUTES_API_TOKEN")

        # 后端配置
        self.backends = {
            "chutes": {
                "url": "https://llm.chutes.ai/v1/chat/completions",
                "model": "deepseek-ai/DeepSeek-V3",
                "requires_key": True
            },
            "ollama": {
                "url": "http://localhost:11434/api/generate",
                "model": "gemma2:2b",  # Using Gemma2 2B - faster, less memory
                "requires_key": False
            },
            "openai": {
                "url": "https://api.openai.com/v1/chat/completions",
                "model": "gpt-4",
                "requires_key": True
            }
        }

        if backend not in self.backends:
            raise ValueError(f"Unsupported backend: {backend}")

        self.config = self.backends[backend]

        # 检查API key
        if self.config["requires_key"] and not self.api_key:
            print(f"⚠️  Warning: {backend} requires API key but none provided")

    async def generate(self, prompt: str, temperature: float = 0.3) -> str:
        """
        调用LLM生成文本

        Args:
            prompt: 输入prompt
            temperature: 采样温度

        Returns:
            生成的文本
        """
        if self.backend == "ollama":
            return await self._call_ollama(prompt, temperature)
        else:
            return await self._call_openai_compatible(prompt, temperature)

    async def _call_openai_compatible(self, prompt: str, temperature: float) -> str:
        """调用OpenAI兼容的API (Chutes, OpenAI)"""
        headers = {
            "Content-Type": "application/json"
        }

        if self.api_key:
            headers["Authorization"] = f"Bearer {self.api_key}"

        data = {
            "model": self.config["model"],
            "messages": [{"role": "user", "content": prompt}],
            "temperature": temperature,
            "max_tokens": 2000
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(
                self.config["url"],
                headers=headers,
                json=data
            ) as response:
                if response.status != 200:
                    error = await response.text()
                    raise Exception(f"LLM API error: {error}")

                result = await response.json()
                return result["choices"][0]["message"]["content"]

    async def _call_ollama(self, prompt: str, temperature: float) -> str:
        """调用本地Ollama"""
        data = {
            "model": self.config["model"],
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": temperature
            }
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(
                self.config["url"],
                json=data
            ) as response:
                if response.status != 200:
                    error = await response.text()
                    raise Exception(f"Ollama error: {error}")

                result = await response.json()
                return result["response"]


# 测试
async def test():
    client = LLMClient(backend="chutes")
    response = await client.generate("Say hello in 3 words")
    print(response)


if __name__ == "__main__":
    asyncio.run(test())
