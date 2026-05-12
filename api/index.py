import sys
import os

# Đảm bảo Vercel có thể tìm thấy các module trong thư mục gốc
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from web_app.main import app

# Export app for Vercel
handler = app
