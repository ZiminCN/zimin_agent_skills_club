#!/usr/bin/env python3
"""Capture front (3:4) and back (9:16) from continuous design canvas."""
import asyncio
from pathlib import Path
from playwright.async_api import async_playwright

BASE = Path(__file__).parent
HTML = BASE / "ireader_case_design.html"

H = 1600
FRONT_W = 1200
BACK_W = 900

OUT_FRONT = BASE / "ireader_case_front.png"
OUT_BACK = BASE / "ireader_case_back.png"
OUT_FULL = BASE / "ireader_case_full.png"

async def main():
    async with async_playwright() as p:
        browser = await p.chromium.launch()
        page = await browser.new_page(viewport={"width": 2200, "height": 1700})
        await page.goto(f"file://{HTML}", wait_until="networkidle")
        await page.wait_for_timeout(4000)

        canvas = await page.query_selector(".continuous-canvas")
        if not canvas:
            print("ERROR: .continuous-canvas not found!")
            await browser.close()
            return

        box = await canvas.bounding_box()
        if not box:
            print("ERROR: canvas bounding_box failed!")
            await browser.close()
            return

        x0, y0 = box["x"], box["y"]

        # Full canvas
        await page.screenshot(path=str(OUT_FULL),
            clip={"x": x0, "y": y0, "width": FRONT_W + BACK_W, "height": H})
        print(f"Full: {OUT_FULL.name}")

        # Front (3:4) — left portion
        await page.screenshot(path=str(OUT_FRONT),
            clip={"x": x0, "y": y0, "width": FRONT_W, "height": H})
        print(f"Front: {OUT_FRONT.name} ({FRONT_W}x{H})")

        # Back (9:16) — right portion
        await page.screenshot(path=str(OUT_BACK),
            clip={"x": x0 + FRONT_W, "y": y0, "width": BACK_W, "height": H})
        print(f"Back: {OUT_BACK.name} ({BACK_W}x{H})")

        await browser.close()
        print("All done!")

asyncio.run(main())
