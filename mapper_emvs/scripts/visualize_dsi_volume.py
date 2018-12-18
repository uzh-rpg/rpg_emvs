import numpy as np
import argparse
import visvis as vv
app = vv.use()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Plot the disparity space image (DSI) using 3D slices')
    parser.add_argument('-i', '--input', default='dsi.npy', type=str,
                        help='path to the NPY file containing the DSI (default: dsi.npy)')
    args = parser.parse_args()

    a = vv.gca()
    a.daspect = 1, -1, 1
    a.daspectAuto = True
    vol = np.load(args.input)
    
    # Reorder axis so that the Z axis points forward instead of up
    vol = np.swapaxes(vol, 0, 1)
    vol = np.flip(vol, axis=0)
    
    t = vv.volshow(vol, renderStyle = 'mip')
    t.colormap = vv.CM_HOT
    
    app.Run()
