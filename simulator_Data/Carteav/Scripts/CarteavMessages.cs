using System;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using Simulator.Bridge;
using Simulator.Bridge.Data.Ros;
using Ros = Simulator.Bridge.Data.Ros;


using Vector3 = UnityEngine.Vector3;

namespace Carteav.Messages
{
    [MessageType("carteav_interfaces/CartPath")]
    public struct CartPathMessage
    {
        public CartPointMessage[] points;
        public string path_id;
        public UInt32 path_length_m;
        public UInt32 path_duration_sec;
        public bool cyclic;
        public string dest_name;
        public byte dest_counter;
        public byte dest_limit;
    }
    
    [MessageType("carteav_interfaces/CartPoint")]
    public struct CartPointMessage
    {
        public Point point;
        public double max_velocity_mps;
        public double req_velocity_mps;
        public Int16 current_eta_sec;
        public bool is_crosswalk;
        public Int16 cross_junction_id;
        public bool is_speed_bumps;
    }

    
    [MessageType("carteav_interfaces/SiteBoundries")]
    public struct SiteBoundriesMessage
    {
        public SiteBoundryMessage[] boundries;
        public int data_version;
    }
    
    [MessageType("carteav_interfaces/SiteBoundry")]
    public struct SiteBoundryMessage
    {
        public uint boundary_id;
        public PolygonMessage[] polygons;
    }

    [MessageType("carteav_interfaces/Polygon")]
    public struct PolygonMessage
    {
        public Point[] points;
    }
    
    [MessageType("carteav_interfaces/BoundaryCross")]
    public struct BoundaryCrossMessage
    {
        public string object_name;
        public Point position;
        public Ros.Vector3 velocity;
        public double time;
        public string boundary_type;
    }
    
    
    [MessageType("carteav_interfaces/Collision")]
    public struct CollisionMessage
    {
        public string object_name;
        public Point position;
        public float yaw_angle;
        public Ros.Vector3 velocity;
        public double time;
    }
    
    
    public class CartPath
    {
        public List<CartPoint> Points;
        public string PathId;
        public UInt32 PathLengthM;
        public UInt32 PathDurationSec;
        public bool Cyclic;
        public string DestName;
        public byte DestCount;
        public byte DestLimiter;

        public CartPath(CartPathMessage pathMessage)
        {
            Points = new List<CartPoint>(pathMessage.points.ToList().ConvertAll(structPoint => new CartPoint(structPoint)));
            PathId = pathMessage.path_id;
            PathLengthM = pathMessage.path_length_m;
            PathDurationSec = pathMessage.path_duration_sec;
            Cyclic = pathMessage.cyclic;
            DestName = pathMessage.dest_name;
            DestCount = pathMessage.dest_counter;
            DestLimiter =pathMessage.dest_limit;
        }
    }

    public class CartPoint
    {
        public Vector3 Point;
        public double MAXVelocityMps;
        public double ReqVelocityMps;
        public Int16 CurrentEtaSec;
        public bool IsCrosswalk;
        public Int16 JunctionId;
        public bool IsSpeedBumps;

        public CartPoint(CartPointMessage pointMessageStruct)
        {
            var v = pointMessageStruct.point; 
            //Point = new Vector3() { x = (float)v.x, y = (float)v.z, z = (float)v.y };
            Point = Converters.ConvertCoordinates(v);
            MAXVelocityMps = pointMessageStruct.max_velocity_mps;
            ReqVelocityMps = pointMessageStruct.req_velocity_mps;
            CurrentEtaSec = pointMessageStruct.current_eta_sec;
            IsCrosswalk = pointMessageStruct.is_crosswalk;
            JunctionId = pointMessageStruct.cross_junction_id;
            IsSpeedBumps = pointMessageStruct.is_speed_bumps;
        }
    }


    public class SiteBoundaries
    {
        public List<SiteBoundry> boundries;
        int data_version;

        public SiteBoundaries(SiteBoundriesMessage boundaries)
        {
            boundries = boundaries.boundries.ToList()
                .ConvertAll(dataBoundary => new SiteBoundry(dataBoundary));
        }

        
    }

    public class SiteBoundry
    {
        public uint boundary_id;
        public List<Polygon> Polygons;
        public SiteBoundry(SiteBoundryMessage boundary)
        {
            Polygons = boundary.polygons.ToList().ConvertAll(dataPolygon => new Polygon(dataPolygon));
        }
       
    }

    public class Polygon
    {
        public List<Vector3> Points;
        
        public Polygon(PolygonMessage polygonMessage)
        {
            Points = polygonMessage.points.ToList()
                .ConvertAll(Converters.ConvertCoordinates);
        }

        
    }

    public class CollisionData
    {
        public string ObjectName;
        public Vector3 Position;
        public float YawAngle;
        public Vector3 Velocity;
        public double Time;
    }
    
    public class BoundaryCross
    {
        public string ObjectName;
        public Vector3 Position;
        public Vector3 Velocity;
        public double Time;
        public MapBoundary.BoundaryType BoundaryType;
    }


    public static class Converters
    {
        /*
           Should be: x=-y`;y=z`; z=x` according to manual:
           https://www.svlsimulator.com/docs/getting-started/conventions/#converting-between-coordinate-systems
       */
        public static Vector3 ConvertCoordinates(Point coords)
        {
            return new Vector3((float)coords.x, (float)coords.z, (float)coords.y);
        }
        
        public static Vector3 ConvertFromVector(Ros.Vector3 v)
        {
            return new Vector3() { x = (float)v.x, y = (float)v.y, z = (float)v.z };
        }
        
        public static Ros.Vector3 ConvertToVector(Vector3 v)
        {
            return new Ros.Vector3() { x = v.x, y = v.y, z = v.z };
        }
        
        public static  Vector3 ConvertFromPoint(Point  v)
        {
            return new Vector3 () { x = (float)v.x, y = (float)v.y, z = (float)v.z };
        }
        
        
        public static Point ConvertToPoint(Vector3 v)
        {
            return new Point() { x = v.x, y = v.y, z = v.z };
        }


        public static BoundaryCrossMessage ConvertBoundaryCross(BoundaryCross cross)
        {
            return new BoundaryCrossMessage()
            {
                object_name = cross.ObjectName,
                position = ConvertToPoint(cross.Position),
                velocity = ConvertToVector(cross.Velocity),
                time = cross.Time,
                boundary_type = cross.BoundaryType.ToString()
            };
        }
        
        public static CollisionMessage ConvertCollision(CollisionData collision)
        {
            return new CollisionMessage()
            {
                object_name = collision.ObjectName,
                position = ConvertToPoint(collision.Position),
                yaw_angle = collision.YawAngle,
                velocity = ConvertToVector(collision.Velocity),
                time = collision.Time,
            };
        }
    }
    
}


