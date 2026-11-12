"""Discover offline video encoding support, without device access."""
import importlib.util,json,shutil
out=dict(ffmpeg=shutil.which('ffmpeg'),ffprobe=shutil.which('ffprobe'))
if importlib.util.find_spec('imageio_ffmpeg'):
    import imageio_ffmpeg
    out['imageio_ffmpeg']=imageio_ffmpeg.get_ffmpeg_exe()
print(json.dumps(out),flush=True)
