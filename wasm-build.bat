call emcc --bind -Os -s ALLOW_MEMORY_GROWTH -s USE_WEBGL2=0 -s USE_LIBPNG=1 msdfgen-svg.cpp -o msdfgen-svg.js
call python compose.py
