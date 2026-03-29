@group(0) @binding(0)
var<storage, read> vertices: array<vec2<f32>>;

@group(0) @binding(1)
var<storage, read> indices: array<u32>;

@group(0) @binding(2)
var<storage, read> values: array<f32>;

struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) value: f32,
};

fn color_map(t_in: f32) -> vec3<f32> {
    let t = clamp(t_in, 0.0, 1.0);
    // blue -> cyan -> yellow -> red
    let c1 = mix(vec3<f32>(0.0, 0.0, 0.6), vec3<f32>(0.0, 0.9, 1.0), smoothstep(0.0, 0.33, t));
    let c2 = mix(c1, vec3<f32>(1.0, 0.9, 0.1), smoothstep(0.33, 0.66, t));
    let c3 = mix(c2, vec3<f32>(0.9, 0.1, 0.0), smoothstep(0.66, 1.0, t));
    return c3;
}

@vertex
fn vs_main(
    @builtin(vertex_index) local_vid: u32,      // 0,1,2
    @builtin(instance_index) tri_id: u32// one instance per triangle
) -> VertexOutput {
    var out: VertexOutput;

    let i = tri_id * 3u + local_vid;
    let v_idx = indices[i];

    let p = vertices[v_idx];                    // expected NDC in [-1,1]
    out.position = vec4<f32>(p, 0.0, 1.0);

    out.value = values[v_idx];
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // in.value is barycentrically interpolated from the 3 triangle vertices
    let rgb = color_map(in.value);
    return vec4<f32>(rgb, 1.0);
}