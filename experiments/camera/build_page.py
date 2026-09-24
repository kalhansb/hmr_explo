#!/usr/bin/env python3
"""page_template.html + page_data.json -> page.html (the published results page)."""
import json, os
HERE = os.path.dirname(os.path.abspath(__file__))
t = open(os.path.join(HERE, "page_template.html")).read()
d = json.load(open(os.path.join(HERE, "page_data.json")))
open(os.path.join(HERE, "page.html"), "w").write(t.replace("__DATA__", json.dumps(d, separators=(",", ":"))))
print("wrote page.html", os.path.getsize(os.path.join(HERE, "page.html")) // 1024, "KB")
