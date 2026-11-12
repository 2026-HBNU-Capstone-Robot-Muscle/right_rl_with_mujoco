import importlib.util
print({name: importlib.util.find_spec(name) is not None for name in ('scipy','cv2','numpy')})
