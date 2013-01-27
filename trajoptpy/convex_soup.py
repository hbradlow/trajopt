import scipy.spatial
import numpy as np
import openravepy as rave
import itertools
import cloudprocpy

def calc_hull(points):
    if len(points) < 3: return points, np.zeros((0,3), int)
    elif len(points) == 3: return points, np.array([[0,1,2]], dtype=int)
    elif len(points) == 4: return points, np.array(list(itertools.combinations(range(4),3)), dtype=int)
    else:
        while True:
            pertpts = points + np.random.randn(*points.shape)*.001
            try:
                delaunay = scipy.spatial.Delaunay(pertpts)
                return delaunay.points, delaunay.convex_hull
            except Exception:
                print "qhull error, returning"
                return None,None

def create_convex_soup(cloud, env, name = "convexsoup", thresh = .03, min_num_vertices = 10):
    xyz = cloud.to2dArray()
    indss = cloudprocpy.convexDecomp(cloud, thresh)
    geom_infos = []
    for (i,inds) in enumerate(indss):
        if len(inds) < min_num_vertices: continue # openrave is slow with a lot of geometries

        origpts = xyz[inds]
        hullpoints, hullinds = calc_hull(origpts)
        if hullpoints is None: 
            print "failed to get convex hull!"
            continue
            
            
        gi = rave.KinBody.GeometryInfo()
        gi._meshcollision = rave.TriMesh(hullpoints, hullinds)
        gi._type = rave.GeometryType.Trimesh
        gi._vAmbientColor = np.random.rand(3)/2
        
        geom_infos.append(gi)

    body = rave.RaveCreateKinBody(env,'')
    body.SetName(name)
    body.InitFromGeometries(geom_infos)
    env.Add(body)

#Same as create_convex_soup, except that the convex soup within a box have different parameter from the rest. The box has half lengths hl_box and transform T.
def create_convex_soup_dynamic(cloud, env, T, hl_box = 0.4):
    cloud_far = cloudprocpy.orientedBoxFilter(cloud, T.astype("float32"), -hl_box, hl_box, -hl_box, hl_box, -hl_box, hl_box, True)
    cloud_near = cloudprocpy.orientedBoxFilter(cloud, T.astype("float32"), -hl_box, hl_box, -hl_box, hl_box, -hl_box, hl_box, False)
    create_convex_soup(cloud_far, env, "convexsoup_far", .03, 10)
    create_convex_soup(cloud_near, env, "convexsoup_near", .01, 4)
