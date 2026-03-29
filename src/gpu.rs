use nannou::prelude::*;
use spade::{DelaunayTriangulation, HasPosition, Triangulation};

pub struct GpuState {
    pub render_pipeline: wgpu::RenderPipeline,
    // uniform_buffer: wgpu::Buffer,
    vertex_buffer: wgpu::Buffer,
    tri_buffer: wgpu::Buffer,
    values_buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,
}

impl GpuState {
    pub fn new<V>(
        device: &wgpu::Device,
        triangulation: &DelaunayTriangulation<V>,
        half_extents: [f32; 2],
    ) -> Self
    where
        V: HasPosition<Scalar = f32>,
    {
        // let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
        //     label: Some("Uniform Buffer"),
        //     size: std::mem::size_of::<Uniforms>() as u64,
        //     usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        //     mapped_at_creation: false,
        // });

        let inv_w = if half_extents[0].abs() > f32::EPSILON {
            1.0 / half_extents[0]
        } else {
            1.0
        };
        let inv_h = if half_extents[1].abs() > f32::EPSILON {
            1.0 / half_extents[1]
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
                f_hand
                    .vertices()
                    .map(|v_hand| u32::try_from(v_hand.index()).expect("vertex index must fit into u32"))
            })
            .collect::<Vec<u32>>();

        let values = (0..vertices.len())
            .map(|_| random_f32())
            .collect::<Vec<_>>();

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Pendulum States Storage Buffer"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        });

        let tri_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Triangle Indices Storage Buffer"),
            contents: bytemuck::cast_slice(&tris),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        });

        let values_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Pendulum States Storage Buffer"),
            contents: bytemuck::cast_slice(&values),
            usage: wgpu::BufferUsages::STORAGE
                | wgpu::BufferUsages::COPY_DST
                | wgpu::BufferUsages::COPY_SRC,
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Render Bind Group Layout"),
            entries: &[
                // Binding 0: Uniforms
                // wgpu::BindGroupLayoutEntry {
                //     binding: 0,
                //     visibility: wgpu::ShaderStages::FRAGMENT,
                //     ty: wgpu::BindingType::Buffer {
                //         ty: wgpu::BufferBindingType::Uniform,
                //         has_dynamic_offset: false,
                //         min_binding_size: None,
                //     },
                //     count: None,
                // },
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
            ],
        });

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Render Bind Group"),
            layout: &bind_group_layout,
            entries: &[
                // wgpu::BindGroupEntry {
                //     binding: 0,
                //     resource: uniform_buffer.as_entire_binding(),
                // },
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
                count: 4,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
        });

        GpuState {
            vertex_buffer,
            tri_buffer,
            values_buffer,
            bind_group,
            render_pipeline,
        }
    }
}
