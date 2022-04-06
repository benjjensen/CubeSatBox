using GLMakie

# color_theme = Theme(fontsize = 10)
# set_theme!(color = :blues)
# cmap = :thermal
# colors = to_colormap(cmap)

# r = LinRange(-1, 1, 100)
# cube = [(x.^2 + y.^2 + z.^2) for x = r, y = r, z = r]
# fig, ax = volume(cube)
# limits!(ax, -200, 200, -200, 200)




fig = Figure()

ax = Axis(fig[1, 1])

# # 2D Version 
# sl_ρ = Slider(fig[2, 1], range = -179:1:180, startvalue = 0)
# sᴺ = [1.0, 0.0]
# c = lift(sl_ρ.value) do ρ
#     r = deg2rad(ρ)
#     Rρ = [cos.(r)  -sin.(r);
#           sin.(r)   cos.(r)]
#     sᴮ = Rρ * sᴺ

#     return [Point2f(0.0, 0.0), Point2f(sᴮ)]
# end
# #############


# 3D Version
sl_ρ = Slider(fig[2, 1], range = -179:1:180, startvalue = 0)
sl_γ = Slider(fig[3, 1], range = -179:1:180, startvalue = 0)
sl_ϕ = Slider(fig[4, 1], range = -179:1:180, startvalue = 0)
sᴺ = [1; 0; 0]

sᴮ = lift(sl_ρ.value) do ρ 
    r = deg2rad(ρ)
    Rρ =  [
        cos.(r) -sin.(r) 0;
        sin.(r)  cos.(r) 0;
        0       0      1
    ]
    # return [Point2f(0.0, 0.0, 0.0), Point2f(Rρ * sᴺ)]
    return Point3f(Rρ * sᴺ)
end 

sᴮ = lift(sl_γ.value) do γ
    g = deg2rad(γ)
    Rγ = [
        cos.(g)   0      sin.(g);
        0        1      0;
        -sin.(g)  0       cos.(g)
    ]
    # return [Point2f(0.0, 0.0, 0.0), Point2f(Rγ * sᴺ)]
    return Point3f(Rγ * sᴺ)
end 

sᴮ = lift(sl_ϕ.value) do ϕ
    f = deg2rad(ϕ)
    @show ϕ
    Rϕ = [
        1       0      0;
        0    cos.(f) -sin.(f);
        0    sin.(f)  cos.(f);
    ]
    # return [Point2f(0.0, 0.0, 0.0), Point2f(Rϕ * sᴺ)]
    return Point3f(Rϕ * sᴺ)
end 

# point = lift(sl_ρ.value, sl_γ.value, sl_ϕ.value) do ρ, γ, ϕ
#     Point2f([deg2rad(ρ), deg2rad(γ)])
# end

# point = Point2f([1, 1])
# point = lift(sl_x.value, sl_y.value) do x, y
#     Point2f0(x, y)
# end


scatter!(sᴮ, color = :red, markersize = 20)

limits!(ax, -5, 5, -5, 5)

fig