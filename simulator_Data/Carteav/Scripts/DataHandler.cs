﻿using System.Collections.Generic;
using System.Linq;
using Carteav.Messages;
using Simulator.Bridge;
using Simulator.ScenarioEditor.Utilities;
using UnityEngine;

namespace Carteav
{
    /// <summary>
    /// Serves as a relay between the MapSensor(on the cart) and preloaded scene objects handling 2D agent collision.
    /// Also responsible for spawning the map boundaries and rendering recieved path on scene.
    /// </summary>
    public class DataHandler : MonoBehaviour
    {
        [SerializeField] private Agent2DCollider agentCollider2D;
        [SerializeField] private MapBoundary boundaryPrefab;
        [SerializeField] private MapBoundary boundaryHolePrefab;
        [SerializeField] private MapBoundary boundary3DPrefab;
        [SerializeField] private MapBoundary boundaryHole3DPrefab;
        [SerializeField] private Transform boundaryOrientation;
        [SerializeField] private PrefabsPools pools;
        [SerializeField] private Material waypointsMaterial;

        /// <summary>
        /// Renders the received path, all that's required is assigning points to PathRenderer.points
        /// </summary>
        public LineRenderer PathRenderer
        {
            get
            {
                if (pathRenderer != null) return pathRenderer;

                pathRenderer = gameObject.GetComponent<LineRenderer>();
                if (pathRenderer == null)
                {
                    pathRenderer = gameObject.AddComponent<LineRenderer>();
                    pathRenderer.material = waypointsMaterial;
                    pathRenderer.useWorldSpace = false;
                    pathRenderer.positionCount = 1;
                    pathRenderer.SetPosition(0, LineRendererPositionOffset);
                    pathRenderer.sortingLayerName = "Ignore Raycast";
                    pathRenderer.widthMultiplier = 0.1f;
                    pathRenderer.generateLightingData = false;
                    pathRenderer.textureMode = LineTextureMode.Tile;
                }

                return pathRenderer;
            }
        }
        
        // Switching between 2D and 3D mode will change the collision method from using 2D colliders and 3D colliders
        public bool Is2DMode
        {
            get { return is2DMode; }
            set
            {
                if (is2DMode != value && currentBoundaries != null && boundariesInUse.Count > 0)
                {
                    Dispose();
                    is2DMode = value;
                    HandleBoundaries(currentBoundaries);
                    agentCollider2D.gameObject.SetActive(is2DMode);
                    return;
                }

                is2DMode = value;
            }
        }
        
        
        
        private bool is2DMode;
        private List<MapBoundary> boundariesInUse = new List<MapBoundary>();
        private Publisher<BoundaryCross> publishBoundaryCross;
        private Transform agentTransform;
        private LineRenderer pathRenderer;
        private Vector3 LineRendererPositionOffset = new Vector3(0.0f, 0.4f, 0.0f);
        private SiteBoundaries currentBoundaries;

        



        public void Setup(Publisher<BoundaryCross> publisherBoundaryCross, Transform agentTransform, bool is2DMode)
        {
            this.agentTransform = agentTransform;
            publishBoundaryCross = publisherBoundaryCross;
            agentCollider2D.Setup(agentTransform);
            agentCollider2D.gameObject.SetActive(is2DMode);
            Is2DMode = is2DMode;

            var orientBoundary = GameObject.Find("BoundaryOrientation");
            if (orientBoundary != null)
            {
                boundaryOrientation = orientBoundary.transform;
            }
        }


        public void SendBoundaryCross(BoundaryCross boundaryCross)
        {
            publishBoundaryCross(boundaryCross);
        }
        


        public void ToggleData(bool isShown)
        {
            foreach (var mapBoundary in boundariesInUse)
            {
                mapBoundary.SetVisible(isShown);
            }

            PathRenderer.enabled = isShown;
        }


        public void Dispose()
        {
            boundariesInUse.ForEach(boundary =>
            {
                boundary.Dispose();
                
                pools.ReturnInstance(boundary.gameObject);
            });
            boundariesInUse.Clear();
        }

        
        

        public void HandlePath(CartPath path, Vector3 offset)
        {
            PathRenderer.positionCount = path.Points.Count;
            for (int i = 0; i < path.Points.Count; i++)
            {
                 Vector3 point = -path.Points[i].Point;
                 point += offset;
                 point.y = offset.y + LineRendererPositionOffset.y;
                 PathRenderer.SetPosition(i, point);
            }
        }
        

        public void HandleBoundaries(SiteBoundaries boundaries)
        {
            currentBoundaries = boundaries;
            Dispose();
            
            GameObject permittedAreaPrefab = Is2DMode ? boundaryPrefab.gameObject : boundary3DPrefab.gameObject;
            GameObject restrictedAreaPrefab =
                Is2DMode ? boundaryHolePrefab.gameObject : boundaryHole3DPrefab.gameObject;
            for (int j = 0; j < boundaries.boundries.Count; j++)
            {
                var mainAreaPolygon = boundaries.boundries[j].Polygons[0];

                MapBoundary mainArea = pools.GetInstance(permittedAreaPrefab).GetComponent<MapBoundary>();
                mainArea.Setup(mainAreaPolygon, Is2DMode, transform,
                    mainArea.Type.ToString(), boundaryOrientation.position, boundaryOrientation.rotation);
                boundariesInUse.Add(mainArea);

                if (Is2DMode)
                {
                    agentCollider2D.transform.parent = mainArea.transform;
                }

                List<Polygon> holes = boundaries.boundries[j].Polygons
                    .GetRange(1, boundaries.boundries[j].Polygons.Count - 1);
                Vector3 restrictedOffset = new Vector3(0, 0.01f, 0);
                for (int i = 0; i < holes.Count; i++)
                {
                    var hole = holes[i];
                    var restrictedArea = pools.GetInstance(restrictedAreaPrefab).GetComponent<MapBoundary>();
                    restrictedArea.Setup(hole, Is2DMode, mainArea.transform,
                        $"{restrictedArea.Type} - {i}", restrictedOffset);
                    boundariesInUse.Add(restrictedArea);
                }
            }
        }
    }
}