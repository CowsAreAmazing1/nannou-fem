use nannou::prelude::*;
use spade::{ConstrainedDelaunayTriangulation, HasPosition, Triangulation};

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct ColorMapUniform {
    colors: [[f32; 4]; 4],
}

impl Default for ColorMapUniform {
    fn default() -> Self {
        Self {
            colors: [
                [0.0, 0.0, 0.6, 1.0],
                [0.0, 0.9, 1.0, 1.0],
                [1.0, 0.9, 0.1, 1.0],
                [0.9, 0.1, 0.0, 1.0],
            ],
        }
    }
}

impl ColorMapUniform {
    fn from_rgb(colors: [[f32; 3]; 4]) -> Self {
        Self {
            colors: [
                [colors[0][0], colors[0][1], colors[0][2], 1.0],
                [colors[1][0], colors[1][1], colors[1][2], 1.0],
                [colors[2][0], colors[2][1], colors[2][2], 1.0],
                [colors[3][0], colors[3][1], colors[3][2], 1.0],
            ],
        }
    }
}

pub struct GpuState {
    pub render_pipeline: wgpu::RenderPipeline,
    pub vertex_buffer: wgpu::Buffer,
    pub tri_buffer: wgpu::Buffer,
    pub values_buffer: wgpu::Buffer,
    pub render_settings_buffer: wgpu::Buffer,
    pub color_map_buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,
    bind_group_layout: wgpu::BindGroupLayout,
    vertex_capacity: usize,
    tri_capacity: usize,
    values_capacity: usize,
}

impl GpuState {
    fn grow_capacity(current: usize, required: usize) -> usize {
        if required <= current {
            return current;
        }
        required.next_power_of_two().max(1)
    }

    fn create_storage_buffer<T: bytemuck::Pod>(
        device: &wgpu::Device,
        label: &str,
        capacity: usize,
    ) -> wgpu::Buffer {
        let size = (capacity * std::mem::size_of::<T>()).max(std::mem::size_of::<T>()) as u64;
        device.create_buffer(&wgpu::BufferDescriptor {
            label: Some(label),
            size,
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        })
    }

    fn recreate_bind_group(&mut self, device: &wgpu::Device) {
        self.bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Render Bind Group"),
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: self.vertex_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: self.tri_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: self.values_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: self.render_settings_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: self.color_map_buffer.as_entire_binding(),
                },
            ],
        });
    }

    pub fn upload_mesh(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        vertices: &[[f32; 2]],
        tris: &[u32],
        values: Option<&[f32]>,
    ) {
        let required_vertex_capacity = vertices.len().max(1);
        let required_tri_capacity = tris.len().max(1);
        let required_values_capacity = values.map_or(0, |v| v.len()).max(1);

        let new_vertex_capacity =
            Self::grow_capacity(self.vertex_capacity, required_vertex_capacity);
        let new_tri_capacity = Self::grow_capacity(self.tri_capacity, required_tri_capacity);
        let new_values_capacity =
            Self::grow_capacity(self.values_capacity, required_values_capacity);

        let mut resized = false;

        if new_vertex_capacity != self.vertex_capacity {
            self.vertex_buffer = Self::create_storage_buffer::<[f32; 2]>(
                device,
                "Pendulum States Storage Buffer",
                new_vertex_capacity,
            );
            self.vertex_capacity = new_vertex_capacity;
            resized = true;
        }

        if new_tri_capacity != self.tri_capacity {
            self.tri_buffer = Self::create_storage_buffer::<u32>(
                device,
                "Triangle Indices Storage Buffer",
                new_tri_capacity,
            );
            self.tri_capacity = new_tri_capacity;
            resized = true;
        }

        if new_values_capacity != self.values_capacity {
            self.values_buffer = Self::create_storage_buffer::<f32>(
                device,
                "Values Storage Buffer",
                new_values_capacity,
            );
            self.values_capacity = new_values_capacity;
            resized = true;
        }

        if resized {
            self.recreate_bind_group(device);
        }

        if !vertices.is_empty() {
            queue.write_buffer(&self.vertex_buffer, 0, bytemuck::cast_slice(vertices));
        }
        if !tris.is_empty() {
            queue.write_buffer(&self.tri_buffer, 0, bytemuck::cast_slice(tris));
        }
        if let Some(values) = values
            && !values.is_empty()
        {
            queue.write_buffer(&self.values_buffer, 0, bytemuck::cast_slice(values));
        }
    }

    pub fn upload_render_settings(&self, queue: &wgpu::Queue, contour_steps: u32, enabled: bool) {
        let settings = [contour_steps.max(1), if enabled { 1 } else { 0 }];
        queue.write_buffer(
            &self.render_settings_buffer,
            0,
            bytemuck::cast_slice(&settings),
        );
    }

    pub fn upload_color_map(&self, queue: &wgpu::Queue, colors: [[f32; 3]; 4]) {
        let uniform = ColorMapUniform::from_rgb(colors);
        queue.write_buffer(&self.color_map_buffer, 0, bytemuck::bytes_of(&uniform));
    }

    pub fn prepare_geometry<V>(
        triangulation: &ConstrainedDelaunayTriangulation<V>,
        window_rect: Rect,
    ) -> (Vec<[f32; 2]>, Vec<u32>)
    where
        V: HasPosition<Scalar = f32>,
    {
        let (_, _, w, h) = window_rect.x_y_w_h();
        let half_w = w * 0.5;
        let half_h = h * 0.5;

        let inv_w = if half_w.abs() > f32::EPSILON {
            1.0 / half_w
        } else {
            1.0
        };
        let inv_h = if half_h.abs() > f32::EPSILON {
            1.0 / half_h
        } else {
            1.0
        };

        let vertices = triangulation
            .vertices()
            .map(|v| {
                let pos = v.position();
                [
                    (pos.x * inv_w).clamp(-1.0, 1.0),
                    (pos.y * inv_h).clamp(-1.0, 1.0),
                ]
            })
            .collect::<Vec<_>>();

        let tris = triangulation
            .inner_faces()
            .flat_map(|f_hand| {
                f_hand.vertices().map(|v_hand| {
                    u32::try_from(v_hand.index()).expect("vertex index must fit into u32")
                })
            })
            .collect::<Vec<u32>>();

        (vertices, tris)
    }

    pub fn new<V>(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        triangulation: &ConstrainedDelaunayTriangulation<V>,
        window_rect: Rect,
        values: Option<Vec<f32>>,
    ) -> Self
    where
        V: HasPosition<Scalar = f32>,
    {
        let (vertices, tris) = GpuState::prepare_geometry(triangulation, window_rect);

        let values = values.unwrap_or_else(|| {
            let num = vertices.len() as f32;
            (0..vertices.len()).map(|i| i as f32 / num).collect()
        });

        let vertex_capacity = vertices.len().max(1);
        let tri_capacity = tris.len().max(1);
        let values_capacity = values.len().max(1);

        let vertex_buffer = Self::create_storage_buffer::<[f32; 2]>(
            device,
            "Pendulum States Storage Buffer",
            vertex_capacity,
        );

        let tri_buffer = Self::create_storage_buffer::<u32>(
            device,
            "Triangle Indices Storage Buffer",
            tri_capacity,
        );

        let values_buffer =
            Self::create_storage_buffer::<f32>(device, "Values Storage Buffer", values_capacity);

        let render_settings_buffer =
            Self::create_storage_buffer::<u32>(device, "Render Settings Buffer", 2);

        let color_map_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Color Map Uniform Buffer"),
            size: std::mem::size_of::<ColorMapUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Render Bind Group Layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Storage { read_only: true },
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Render Bind Group"),
            layout: &bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: vertex_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: tri_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: values_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: render_settings_buffer.as_entire_binding(),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: color_map_buffer.as_entire_binding(),
                },
            ],
        });

        let render_shader = include_str!("render.wgsl");
        let render_module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Render Shader"),
            source: wgpu::ShaderSource::Wgsl(render_shader.into()),
        });

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[&bind_group_layout],
                push_constant_ranges: &[],
            });

        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &render_module,
                entry_point: "vs_main",
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &render_module,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: Frame::TEXTURE_FORMAT,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
        });

        let mut state = GpuState {
            vertex_buffer,
            tri_buffer,
            values_buffer,
            render_settings_buffer,
            color_map_buffer,
            bind_group,
            bind_group_layout,
            render_pipeline,
            vertex_capacity,
            tri_capacity,
            values_capacity,
        };

        state.upload_mesh(device, queue, &vertices, &tris, Some(&values));
        let default_colors = ColorMapUniform::default();
        queue.write_buffer(
            &state.color_map_buffer,
            0,
            bytemuck::bytes_of(&default_colors),
        );

        state
    }
}
