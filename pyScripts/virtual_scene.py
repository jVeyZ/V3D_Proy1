"""
virtual_scene.py - Escena virtual 3D con Open3D (Gemelo Digital).

Apartado 1.1: Crear y mostrar una escena virtual que sea un gemelo digital 
              de la escena real. Muestra al menos un objeto real (pelota) 
              y uno virtual (hoyo).
"""

import numpy as np

try:
    import open3d as o3d # type: ignore
    OPEN3D_AVAILABLE = True
except ImportError:
    OPEN3D_AVAILABLE = False
    print("[VirtualScene] Open3D no disponible. Se usará solo vista AR.")

import config


class VirtualScene:
    """
    Escena virtual 3D usando Open3D que replica el escenario real.
    
    Elementos:
      - Mesa/plano de trabajo (objeto real → representación virtual)
      - Pelota (objeto real → posición actualizada desde la cámara)
      - Hoyo (objeto virtual)
      - Bandera (objeto virtual)
      - Trayectoria de la pelota (virtual)
      - Obstáculos (virtuales, opcionales)
    """

    def __init__(self):
        self.vis = None
        self._geometries = {}
        self._ball_position = np.array([30.0, 20.0, config.BALL_REAL_RADIUS_CM])
        self._hole_position = np.array([45.0, 20.0, 0.0])
        self._trail_points = []
        self._is_running = False
        self._thread = None
        self._update_needed = False
        self._scale = config.SCENE_SCALE

    def initialize(self, hole_position=None):
        """Inicializa la escena virtual con todos los elementos."""
        if not OPEN3D_AVAILABLE:
            print("[VirtualScene] Open3D no disponible, saltando inicialización.")
            return False

        if hole_position is not None:
            self._hole_position = np.array([hole_position[0], hole_position[1], 0.0])

        self._create_geometries()
        self._start_visualization()
        return True

    def _create_geometries(self):
        """Crea todas las geometrías de la escena."""
        s = self._scale
        w = config.PLAY_AREA_WIDTH
        h = config.PLAY_AREA_HEIGHT

        # --- MESA (Green) ---
        # Dar más espesor visible a la mesa para que se perciba en 3D
        table = o3d.geometry.TriangleMesh.create_box(
            width=w * s, height=h * s, depth=5.0 * s
        )
        # Trasladar la mesa hacia abajo la mitad de su espesor
        table.translate([0, 0, -2.5 * s])
        table.paint_uniform_color(config.COLOR_TABLE)
        table.compute_vertex_normals()
        self._geometries['table'] = table

        # --- BORDE DE LA MESA ---
        border_thickness = 3.0 * s
        border_height = 6.0 * s
        borders = []
        # Borde inferior
        b = o3d.geometry.TriangleMesh.create_box(
            w * s + 2 * border_thickness, border_thickness, border_height
        )
        b.translate([-border_thickness, -border_thickness, -2.5 * s])
        borders.append(b)
        # Borde superior
        b = o3d.geometry.TriangleMesh.create_box(
            w * s + 2 * border_thickness, border_thickness, border_height
        )
        b.translate([-border_thickness, h * s, -2.5 * s])
        borders.append(b)
        # Borde izquierdo
        b = o3d.geometry.TriangleMesh.create_box(
            border_thickness, h * s, border_height
        )
        b.translate([-border_thickness, 0, -2.5 * s])
        borders.append(b)
        # Borde derecho
        b = o3d.geometry.TriangleMesh.create_box(
            border_thickness, h * s, border_height
        )
        b.translate([w * s, 0, -2.5 * s])
        borders.append(b)

        for i, border in enumerate(borders):
            border.paint_uniform_color(config.COLOR_TABLE_BORDER)
            border.compute_vertex_normals()
            self._geometries[f'border_{i}'] = border

        # --- PELOTA (objeto real representado virtualmente) ---
        ball = o3d.geometry.TriangleMesh.create_sphere(
            radius=config.BALL_REAL_RADIUS_CM * s, resolution=20
        )
        ball_pos = self._ball_position * s
        ball.translate(ball_pos)
        ball.paint_uniform_color(config.COLOR_BALL)
        ball.compute_vertex_normals()
        self._geometries['ball'] = ball

        # --- HOYO (objeto virtual) ---
        # Hacer el hoyo más profundo y visible
        hole = o3d.geometry.TriangleMesh.create_cylinder(
            radius=config.HOLE_RADIUS_CM * s, height=10.0 * s,
            resolution=60
        )
        hole_pos = self._hole_position * s
        # Colocar el cilindro centrado y hundido parcialmente en la mesa
        hole.translate([hole_pos[0], hole_pos[1], -5.0 * s])
        hole.paint_uniform_color(config.COLOR_HOLE)
        hole.compute_vertex_normals()
        self._geometries['hole'] = hole

        # --- BANDERA (objeto virtual) ---
        # Palo (más alto y visible)
        pole = o3d.geometry.TriangleMesh.create_cylinder(
            radius=0.25 * s, height=15 * s, resolution=16
        )
        # Elevar la base del palo sobre la superficie
        pole.translate([hole_pos[0], hole_pos[1], 0.5 * s])
        pole.paint_uniform_color(config.COLOR_FLAG_POLE)
        pole.compute_vertex_normals()
        self._geometries['pole'] = pole

        # Bandera triangular (en la parte superior del palo)
        flag = o3d.geometry.TriangleMesh()
        flag_h = 8 * s
        flag_w = 8 * s
        flag_base_z = 15 * s + 0.5 * s
        vertices = np.array([
            [hole_pos[0], hole_pos[1], flag_base_z],
            [hole_pos[0] + flag_w, hole_pos[1], flag_base_z + flag_h / 2],
            [hole_pos[0], hole_pos[1], flag_base_z + flag_h],
        ])
        triangles = np.array([[0, 1, 2]])
        flag.vertices = o3d.utility.Vector3dVector(vertices)
        flag.triangles = o3d.utility.Vector3iVector(triangles)
        flag.paint_uniform_color(config.COLOR_FLAG)
        flag.compute_vertex_normals()
        self._geometries['flag'] = flag

        # --- OBSTÁCULOS (virtuales) ---
        for i, (ox, oy, orad) in enumerate(config.OBSTACLES):
            obs = o3d.geometry.TriangleMesh.create_cylinder(
                radius=orad * s, height=3 * s, resolution=20
            )
            obs.translate([ox * s, oy * s, 0])
            obs.paint_uniform_color(config.COLOR_OBSTACLE)
            obs.compute_vertex_normals()
            self._geometries[f'obstacle_{i}'] = obs

        # --- TRAYECTORIA (línea) ---
        trail = o3d.geometry.LineSet()
        self._geometries['trail'] = trail

        # --- EJES DE REFERENCIA ---
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=5 * s, origin=[0, 0, 0]
        )
        self._geometries['axes'] = axes

    def initialize(self, hole_position=None):
        """Inicializa la escena virtual con todos los elementos."""
        if not OPEN3D_AVAILABLE:
            print("[VirtualScene] Open3D no disponible, saltando inicialización.")
            return False

        if hole_position is not None:
            self._hole_position = np.array([hole_position[0], hole_position[1], 0.0])

        self._create_geometries()

        # Crear visualizador en el hilo principal y añadir geometrías
        try:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window(
                window_name="Gemelo Digital - Mini Golf",
                width=800, height=600
            )

            # Añadir todas las geometrías
            for geom in self._geometries.values():
                self.vis.add_geometry(geom)

                # Configurar cámara (vista oblicua para enfatizar el 3D)
            ctr = self.vis.get_view_control()
            ctr.set_zoom(0.6)
            # Un front con componente positiva x y negativa y y z ligeramente negativa
            # para obtener un ángulo oblicuo y elevar la mirada ligeramente
            ctr.set_front([0.5, -1.0, -0.7])
            ctr.set_lookat([
                config.PLAY_AREA_WIDTH * self._scale / 2,
                config.PLAY_AREA_HEIGHT * self._scale / 2,
                0.02  # mirar un poco por encima del plano para dar profundidad
            ])
            ctr.set_up([0, 0, 1])

            # Opciones de renderizado
            opt = self.vis.get_render_option()
            opt.background_color = np.array([0.18, 0.18, 0.28])
            opt.mesh_show_back_face = True
            opt.point_size = 3
            opt.light_on = True

            self._is_running = True
            self._update_needed = True
            return True
        except Exception as e:
            print(f"[VirtualScene] Error iniciando visualizador Open3D: {e}")
            self.vis = None
            self._is_running = False
            return False

    def step(self):
        """Actualizar el visualizador (debe llamarse desde el bucle principal)."""
        if not OPEN3D_AVAILABLE or self.vis is None:
            return

        if self._update_needed:
            for geom in self._geometries.values():
                self.vis.update_geometry(geom)
            self._update_needed = False

        self.vis.poll_events()
        self.vis.update_renderer()

    def stop(self):
        """Detiene la visualización."""
        self._is_running = False
        if self.vis is not None:
            try:
                self.vis.destroy_window()
            except Exception:
                pass
            self.vis = None

    def update_ball_position(self, world_pos_cm):
        """
        Actualiza la posición de la pelota en la escena virtual.
        
        Args:
            world_pos_cm: np.array([X, Y]) en cm
        """
        if not OPEN3D_AVAILABLE or 'ball' not in self._geometries:
            return

        s = self._scale
        new_pos_3d = np.array([
            world_pos_cm[0] * s,
            world_pos_cm[1] * s,
            config.BALL_REAL_RADIUS_CM * s
        ])
        old_pos_3d = self._ball_position * s

        # Mover la pelota
        translation = new_pos_3d - old_pos_3d
        self._geometries['ball'].translate(translation)

        self._ball_position = np.array([
            world_pos_cm[0], world_pos_cm[1], config.BALL_REAL_RADIUS_CM
        ])

        # Añadir punto a la trayectoria
        self._trail_points.append(new_pos_3d.copy())
        self._update_trail()

        self._update_needed = True

    def _update_trail(self):
        """Actualiza la línea de trayectoria."""
        if len(self._trail_points) < 2:
            return

        trail = self._geometries.get('trail')
        if trail is None:
            return

        max_trail = 200
        if len(self._trail_points) > max_trail:
            self._trail_points = self._trail_points[-max_trail:]

        points = np.array(self._trail_points)
        lines = [[i, i + 1] for i in range(len(points) - 1)]
        colors = [config.COLOR_TRAIL for _ in lines]

        trail.points = o3d.utility.Vector3dVector(points)
        trail.lines = o3d.utility.Vector2iVector(lines)
        trail.colors = o3d.utility.Vector3dVector(colors)

    def update_hole_position(self, world_pos_cm):
        """Actualiza la posición del hoyo virtual."""
        if not OPEN3D_AVAILABLE:
            return

        s = self._scale
        new_pos = np.array([world_pos_cm[0] * s, world_pos_cm[1] * s, 0])
        old_pos = np.array([self._hole_position[0] * s,
                            self._hole_position[1] * s, 0])
        translation = new_pos - old_pos

        for key in ['hole', 'pole', 'flag']:
            if key in self._geometries:
                self._geometries[key].translate(translation)

        self._hole_position = np.array([world_pos_cm[0], world_pos_cm[1], 0.0])
        self._update_needed = True

    def clear_trail(self):
        """Limpia la trayectoria."""
        self._trail_points = []
        if 'trail' in self._geometries:
            trail = self._geometries['trail']
            trail.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            trail.lines = o3d.utility.Vector2iVector(np.zeros((0, 2), dtype=int))
        self._update_needed = True

    def show_ball_in_hole(self):
        """Anima la pelota 'cayendo' en el hoyo."""
        if not OPEN3D_AVAILABLE or 'ball' not in self._geometries:
            return

        s = self._scale
        # Mover la pelota al centro del hoyo y un poco hacia abajo
        target = np.array([
            self._hole_position[0] * s,
            self._hole_position[1] * s,
            -config.BALL_REAL_RADIUS_CM * s
        ])
        current = self._ball_position * s
        self._geometries['ball'].translate(target - current)
        self._ball_position = target / s
        self._update_needed = True

    # Legacy thread-based stop removed; the active stop() implementation above
    # performs a safe destroy of the Open3D window and cleanup.


class SimpleVirtualScene:
    """
    Escena virtual simplificada que renderiza a una imagen OpenCV.
    Se usa como alternativa cuando Open3D no está disponible o se 
    prefiere una vista 2D superior.
    """

    def __init__(self, width=600, height=400):
        self.width = width
        self.height = height
        self.ball_pos = None
        self.hole_pos = None
        self.trail = []
        self.obstacles = config.OBSTACLES
        self._scale_x = width / config.PLAY_AREA_WIDTH
        self._scale_y = height / config.PLAY_AREA_HEIGHT

    def _world_to_view(self, world_pt):
        """Convierte coordenadas mundo (cm) a coordenadas de la vista."""
        x = int(world_pt[0] * self._scale_x)
        y = int(world_pt[1] * self._scale_y)
        return (x, y)

    def _world_radius_to_view(self, radius_cm):
        return int(radius_cm * (self._scale_x + self._scale_y) / 2)

    def update_ball(self, world_pos):
        self.ball_pos = world_pos
        if world_pos is not None:
            self.trail.append(world_pos.copy())
            if len(self.trail) > 300:
                self.trail = self.trail[-300:]

    def update_hole(self, world_pos):
        self.hole_pos = world_pos

    def clear_trail(self):
        self.trail = []

    def render(self):
        """Renderiza la escena virtual como imagen OpenCV."""
        import cv2

        img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Fondo verde (mesa)
        img[:] = (40, 140, 40)

        # Borde
        cv2.rectangle(img, (0, 0), (self.width - 1, self.height - 1),
                       (30, 80, 200), 3)

        # Obstáculos
        for (ox, oy, orad) in self.obstacles:
            center = self._world_to_view(np.array([ox, oy]))
            r = self._world_radius_to_view(orad)
            cv2.circle(img, center, r, (0, 0, 180), -1)
            cv2.circle(img, center, r, (0, 0, 100), 2)

        # Trayectoria
        if len(self.trail) > 1:
            for i in range(1, len(self.trail)):
                pt1 = self._world_to_view(self.trail[i - 1])
                pt2 = self._world_to_view(self.trail[i])
                alpha = i / len(self.trail)
                color = (0, int(255 * alpha), int(255 * alpha))
                cv2.line(img, pt1, pt2, color, 1)

        # Hoyo
        if self.hole_pos is not None:
            center = self._world_to_view(self.hole_pos)
            r = self._world_radius_to_view(config.HOLE_RADIUS_CM)
            cv2.circle(img, center, r, (50, 50, 50), -1)
            cv2.circle(img, center, r, (0, 0, 0), 2)
            # Bandera
            cv2.line(img, center, (center[0], center[1] - 30), (200, 200, 200), 2)
            pts = np.array([
                [center[0], center[1] - 30],
                [center[0] + 15, center[1] - 25],
                [center[0], center[1] - 20]
            ], dtype=np.int32)
            cv2.fillPoly(img, [pts], (0, 0, 255))

        # Pelota
        if self.ball_pos is not None:
            center = self._world_to_view(self.ball_pos)
            r = self._world_radius_to_view(config.BALL_REAL_RADIUS_CM)
            cv2.circle(img, center, max(r, 5), (0, 130, 255), -1)
            cv2.circle(img, center, max(r, 5), (0, 100, 200), 2)

        # Título
        cv2.putText(img, "GEMELO DIGITAL - Vista Superior",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        return img