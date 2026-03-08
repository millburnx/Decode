{
  "startPoint": {
    "x": 89,
    "y": 8,
    "heading": "linear",
    "startDeg": 90,
    "endDeg": 180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-n7ocftrxevf",
      "name": "Intake HP",
      "endPoint": {
        "x": 132,
        "y": 9,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#D859B9",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mmhj7jn4-mhkou7",
      "name": "Shoot HP",
      "endPoint": {
        "x": 89,
        "y": 9,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#5D9C87",
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
      "lineId": "line-n7ocftrxevf"
    },
    {
      "kind": "path",
      "lineId": "mmhj7jn4-mhkou7"
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
    "onionNextPointOnly": false,
    "showHeadingArrow": false,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1
  },
  "version": "1.2.1",
  "timestamp": "2026-03-08T09:11:41.938Z"
}