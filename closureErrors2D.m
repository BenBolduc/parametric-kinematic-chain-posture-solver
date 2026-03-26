{
  "bodies": [
    {
      "name": "Frame",
      "points": [
        {"id": "P1", "location": [0, 0, 0]},
        {"id": "P4", "location": [0.3, 0, 0]},
        {"id": "P5", "location": [0.6, 0, 0]}
      ]
    },
    {
      "name": "Crank1",
      "points": [
        {"id": "P1", "location": [0, 0, 0]},
        {"id": "P2", "location": [0.1, 0, 0]}
      ]
    },
    {
      "name": "Coupler1",
      "points": [
        {"id": "P2", "location": [0, 0, 0]},
        {"id": "P3", "location": [0.25, 0, 0]}
      ]
    },
    {
      "name": "MiddleLink",
      "points": [
        {"id": "P4", "location": [0, 0, 0]},
        {"id": "P3", "location": [0.2, 0, 0]},
        {"id": "P6", "location": [0.1, 0.1, 0]}
      ]
    },
    {
      "name": "Coupler2",
      "points": [
        {"id": "P6", "location": [0, 0, 0]},
        {"id": "P7", "location": [0.35, 0, 0]}
      ]
    },
    {
      "name": "Follower",
      "points": [
        {"id": "P5", "location": [0, 0, 0]},
        {"id": "P7", "location": [0.2, 0, 0]}
      ]
    }
  ],
  "joints": [
    {"label": "P1", "bodies": ["Frame", "Crank1"], "type": "revolute", "axis": [0, 0, 1]},
    {"label": "P2", "bodies": ["Crank1", "Coupler1"], "type": "revolute", "axis": [0, 0, 1]},
    {"label": "P3", "bodies": ["Coupler1", "MiddleLink"], "type": "revolute", "axis": [0, 0, 1]},
    {"label": "P4", "bodies": ["Frame", "MiddleLink"], "type": "revolute", "axis": [0, 0, 1]},
    {"label": "P6", "bodies": ["MiddleLink", "Coupler2"], "type": "revolute", "axis": [0, 0, 1]},
    {"label": "P7", "bodies": ["Coupler2", "Follower"], "type": "revolute", "axis": [0, 0, 1]},
    {"label": "P5", "bodies": ["Follower", "Frame"], "type": "revolute", "axis": [0, 0, 1]}
  ]
}