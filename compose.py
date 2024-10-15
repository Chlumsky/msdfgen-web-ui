
import base64

with open('body.html', 'r') as f:
    body = f.read()
with open('msdfgen-svg.js', 'r') as f:
    js = f.read().strip()
with open('msdfgen-svg.wasm', 'rb') as f:
    wasm = bytearray(f.read())

failFuncSetup = 'uploadGeneratedDistanceField = failUploadGeneratedDistanceField; encodeGeneratedDistanceField = failEncodeGeneratedDistanceField; immediateGlSetup = noImmediateGlSetup;'

index = body.replace('/*@FAIL_FUNCTION_PLACEHOLDER@*/', failFuncSetup).replace('/*@MODULE_PLACEHOLDER@*/', js.replace('"msdfgen-svg.wasm"', '"data:application/wasm;base64,'+base64.b64encode(wasm).decode('ascii')+'"'))

with open('index.html', 'w') as f:
    f.write(index)
