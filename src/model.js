// MAiRA 7M kinematic + dynamic model.
// Source: NEURA Robotics' own public robot description, published on PyPI as
// neura-rtb-data 0.0.1 (rtbdata/xacro/neura_description/robots/maira7M*.xacro,
// authored by Jens Temminghoff for neura-roboticstoolbox). Values are copied
// verbatim from maira7M_kinematic_parameters_nominal.xacro,
// maira7M_dynamic_parameters_nominal.xacro and maira7M_limits.xacro.
// Lengths in m, masses in kg, inertias in kg m^2, torques in N m, speeds in rad/s.

const H = Math.PI / 2;

export const MAIRA7M = {
  name: 'MAiRA 7M',
  source: 'neura-rtb-data 0.0.1 (PyPI), neura_description/robots/maira7M*.xacro',
  // fixed root joint: rpy (0, 0, 1.5708)
  root: { xyz: [0, 0, 0], rpy: [0, 0, 1.5708] },
  link0: { mass: 2.133, com: [0, 0, 0], inertia: [0, 0, 0, 0, 0, 0] },
  // joints 1..7: origin (relative to parent link frame) + local axis
  joints: [
    { name: 'A1', xyz: [0, 0, 0], rpy: [H, 0, 0], axis: [0, 1, 0], lower: -3.14159, upper: 3.14159, effort: 432, velocity: 2.0944 },
    { name: 'A2', xyz: [0, 0.438, 0], rpy: [0, 0, 0], axis: [1, 0, 0], lower: -2.0944, upper: 2.0944, effort: 432, velocity: 2.0944 },
    { name: 'A3', xyz: [0, 0, 0], rpy: [0, 0, 0], axis: [0, 1, 0], lower: -3.14159, upper: 3.14159, effort: 217, velocity: 2.618 },
    { name: 'A4', xyz: [0, 0.7, 0], rpy: [0, 0, 4.8965276278067904e-12], axis: [1, 0, 0], lower: -2.618, upper: 2.618, effort: 217, velocity: 2.618 },
    { name: 'A5', xyz: [0, 0, 0], rpy: [0, 0, -4.8965276278067904e-12], axis: [0, 1, 0], lower: -3.14159, upper: 3.14159, effort: 57, velocity: 3.8397 },
    { name: 'A6', xyz: [0, 0.7, 0], rpy: [0, 0, 4.8965276278067904e-12], axis: [1, 0, 0], lower: -3.14159, upper: 3.14159, effort: 57, velocity: 3.8397 },
    { name: 'A7', xyz: [0, 0, 0], rpy: [0, 0, -4.8965276278067904e-12], axis: [0, 1, 0], lower: -3.14159, upper: 3.14159, effort: 37, velocity: 3.14159 },
  ],
  // links 1..7 inertial parameters (com in link frame; inertia [ixx, ixy, ixz, iyy, iyz, izz] about com)
  links: [
    { mass: 10.458, com: [0.013845, 0.316366, 0.0], inertia: [0.1615, -0.0175, 0.0, 17.5, 0.0, 0.1942] },
    { mass: 14.8, com: [-0.05, 0.06, -0.035], inertia: [17.5, 0.0011, 0.0, 0.1615, 0.0, 0.12] },
    { mass: 11.0, com: [-0.01, 0.46, 0.0], inertia: [0.3927, -0.0251, 0.0, 3.5, 0.0, 0.4118] },
    { mass: 5.5, com: [-0.03, -0.01, 0.0], inertia: [3.5, 0.0006, 0.0, 0.0113, 0.0, 0.0215] },
    { mass: 8.0, com: [-0.06, 0.37, 0.0], inertia: [0.3098, -0.0624, 0.0, 0.5, 0.0, 0.3218] },
    { mass: 4.5, com: [-0.02, -0.04, -0.01], inertia: [0.5, -0.0013, 0.0, 0.0085, 0.0, 0.0126] },
    { mass: 1.173, com: [0.0, 0.165, 0.0], inertia: [0.0024, 0.0, 0.0, 0.3, 0.0, 0.0024] },
  ],
  // fixed link7 -> flange
  flange: { xyz: [0, 0.115, 0], rpy: [-H, -H, 0] },
  // Datasheet (MAiRA user manual 09/2024, section 7.2): key switch I = cobot speed <= 250 mm/s, II = full speed up to 4.5 m/s.
  cobotSpeed: 0.25,
  fullSpeed: 4.5,
};

// Contact points the planner and the verifier watch: points on the housing that can touch a person.
// link index: 0 = base, 1..7 = link after joint i, 8 = tool (flange frame). Radius = housing radius (m),
// read off the published collision meshes (Link*_body.stl bounding boxes).
export const CONTACT_POINTS = [
  { id: 'shoulder', label: 'Shoulder (A2)', link: 2, p: [0, 0, 0], r: 0.10 },
  { id: 'upperarm', label: 'Upper arm', link: 3, p: [0, 0.35, 0], r: 0.07 },
  { id: 'elbow', label: 'Elbow (A4)', link: 3, p: [0, 0.7, 0], r: 0.075 },
  { id: 'forearm', label: 'Forearm', link: 5, p: [0, 0.35, 0], r: 0.065 },
  { id: 'wrist', label: 'Wrist (A6)', link: 5, p: [0, 0.7, 0], r: 0.06 },
  { id: 'flange', label: 'Flange', link: 8, p: [0, 0, 0], r: 0.05 },
  { id: 'tcp', label: 'Gripper tip (TCP)', link: 8, p: [0, 0, 'tool'], r: 0.03 },
];
