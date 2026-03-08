{
  "startPoint": {
    "x": 89,
    "y": 8,
    "heading": "linear",
    "startDeg": 0,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-3f3eu3wu70t",
      "name": "Path 1",
      "endPoint": {
        "x": 89,
        "y": 16,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#A65BAA",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm6bfbee-ggi4vi",
      "endPoint": {
        "x": 130,
        "y": 9,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#7A886B",
      "name": "Path 6",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm6aqjaj-21egfm",
      "name": "Path 2",
      "endPoint": {
        "x": 89,
        "y": 16,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#C55D67",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm6arbgf-9yaze0",
      "name": "Path 3",
      "endPoint": {
        "x": 128,
        "y": 36,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [
        {
          "x": 89,
          "y": 36
        },
        {
          "x": 89,
          "y": 36
        }
      ],
      "color": "#95B7C8",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm6asbfc-oyy8cf",
      "name": "Path 4",
      "endPoint": {
        "x": 89,
        "y": 16,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#8B6D85",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm6asora-hyabbh",
      "name": "Path 5",
      "endPoint": {
        "x": 120,
        "y": 16,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#6C9B6B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-3f3eu3wu70t"
    },
    {
      "kind": "path",
      "lineId": "mm6bfbee-ggi4vi"
    },
    {
      "kind": "path",
      "lineId": "mm6aqjaj-21egfm"
    },
    {
      "kind": "path",
      "lineId": "mm6arbgf-9yaze0"
    },
    {
      "kind": "path",
      "lineId": "mm6asbfc-oyy8cf"
    },
    {
      "kind": "path",
      "lineId": "mm6asora-hyabbh"
    }
  ],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 58,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 14,
    "rHeight": 16.8,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "decode.webp",
    "robotImage": "/robot.png",
    "theme": "auto",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false
  },
  "version": "1.2.1",
  "timestamp": "2026-02-28T12:47:48.966Z"
}
