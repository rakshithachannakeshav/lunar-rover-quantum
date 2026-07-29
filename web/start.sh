#!/usr/bin/env bash
cd "$(dirname "$0")"
echo ""
echo "  Lunar Rover Web Sim"
echo "  Open: http://localhost:8080"
echo "  Press Ctrl+C to stop"
echo ""
if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "http://localhost:8080" 2>/dev/null &
elif command -v wslview >/dev/null 2>&1; then
  wslview "http://localhost:8080" 2>/dev/null &
fi
exec python3 -m http.server 8080
