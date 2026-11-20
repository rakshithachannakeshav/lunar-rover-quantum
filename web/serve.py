"""Serve the web/ folder like `python -m http.server`, but tell the browser not
to cache anything, so an edited page is never replaced by a stale copy.

Usage:  python serve.py [port]      (default port 8080)
"""
import functools
import http.server
import os
import sys


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, must-revalidate")
        self.send_header("Expires", "0")
        super().end_headers()


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
    here = os.path.dirname(os.path.abspath(__file__))
    handler = functools.partial(NoCacheHandler, directory=here)
    with http.server.ThreadingHTTPServer(("", port), handler) as httpd:
        print("Serving %s at http://localhost:%d  (Ctrl+C to stop)" % (here, port))
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
