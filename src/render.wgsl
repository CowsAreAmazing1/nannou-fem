@group(0) @binding(0)
var<storage, read> vertices: array<vec2<f32>>;

@group(0) @binding(1)
var<storage, read> indices: array<u32>;

@group(0) @binding(2)
var<storage, read> values: array<f32>;

@group(0) @binding(3)
var<storage, read> render_settings: array<u32>;

struct ColorMapUniform {
    colors: array<vec4<f32>, 4>,
};

@group(0) @binding(4)
var<uniform> color_map_uniform: ColorMapUniform;

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) value: f32,
};

fn color_map(t_in: f32) -> vec3<f32> {
    let b1 = color_map_uniform.colors[0].xyz;
    let b2 = color_map_uniform.colors[1].xyz;
    let b3 = color_map_uniform.colors[2].xyz;
    let b4 = color_map_uniform.colors[3].xyz;

    let t = clamp(t_in, 0.0, 1.0);
    // softer gradient: light blue -> cyan -> light yellow -> light red
    let c1 = mix(b1, b2, smoothstep(0.0, 0.33, t));
    let c2 = mix(c1, b3, smoothstep(0.33, 0.66, t));
    let c3 = mix(c2, b4, smoothstep(0.66, 1.0, t));
    return c3;
}

fn apply_contours(t_in: f32, steps: u32, enabled: bool) -> f32 {
    if !enabled || steps < 2u {
        return t_in;
    }

    let n = f32(steps - 1u);
    return round(clamp(t_in, 0.0, 1.0) * n) / n;
}

@vertex
fn vs_main(
    @builtin(vertex_index) local_vid: u32, // 0,1,2
    @builtin(instance_index) tri_id: u32,  // one instance per triangle
) -> VertexOutput {
    var out: VertexOutput;

    let i = tri_id * 3u + local_vid;
    let v_idx = indices[i];

    let p = vertices[v_idx];
    out.position = vec4<f32>(p, 0.0, 1.0);

    out.value = values[v_idx];
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let steps = render_settings[0];
    let enabled = render_settings[1] != 0u;
    let t = apply_contours(in.value, steps, enabled);
    let rgb = color_map(t);
    return vec4<f32>(rgb, 1.0);
}