# [CubeSatBox/PerformanceEvaluation/calibrate_diodes.jl]

""" 
      Used to both evaluate the sun vector estimation and to calibrate the diodes.
    Helpful for comparing the estimation errors before and after calibration. 
    Uses batch estimation (as opposed to a recursive version with MEKF) for 
    convenience. 

    Has a lot of functions because I played around with a lot of things.
"""

# using StaticArrays, SatelliteDynamics, EarthAlbedo 
using LinearAlgebra, Plots, JLD2 
# using LibSerialPort, ForwardDiff


path = "../SensorCalibration/src/MissionSim/"
include(path * "quaternions.jl")

include(path * "CustomStructs.jl");         using .CustomStructs  # Needed to load without warnings




""" Computes the angular distance between two vectors, in degrees """
function get_err(v₁, v₂) 
    """ Gets the angle between two vectors """

    v₁ /= norm(v₁)
    v₂ /= norm(v₂)
    dp = v₁' * v₂ 
    inner = ( (dp >  1.0) && (dp ≈  1.0)) ?  1.0 :          # Deal with numerical errors
            ( (dp < -1.0) && (dp ≈ -1.0)) ? -1.0 : dp 

    err = rad2deg(acos(inner))
    return err
end 

""" 
      Calculates surface normals and solves for the sun vector using a pseudo-inverse. 
    Allows for non-parallel surface normals, and works the best. 
"""
function estimate_sun_vector(Ĩ, azi, elev)
    
    ns = zeros(eltype(Ĩ),  6, 3) 
    for i = 1:6 
        ns[i, :] .= n(azi[i], elev[i])
    end 

    ŝ = (ns' * ns) \ (ns' * Ĩ)
    return ŝ / norm(ŝ)
end

""" Tries to force the vectors to be parallel. Works ok """
function estimate_sun_vector_2(Ĩ, azi, elev)
    nxp = n(azi[1], elev[1]); nxm = n(azi[2], elev[2])
    nⱼ = (nxp .+ (nxm .* [-1, 1, 1])) / 2  # Average out non-parallel components 
    nⱼ /= norm(nⱼ)

    nyp = n(azi[3], elev[3]); nym = n(azi[4], elev[4])
    nₖ = (nyp .+ (nym .* [1, -1, 1])) / 2  # Average out non-parallel components
    nₖ /= norm(nₖ)

    nzp = n(azi[5], elev[5]); nzm = n(azi[6], elev[6])
    nₗ = (nzp .+ (nzm .* [1, 1, -1])) / 2  # Average out non-parallel components
    nₗ /= norm(nₗ)

    ns = [nⱼ'; nₖ'; nₗ']
    
    ŝ = ns \ [Ĩ[1] - Ĩ[2]; Ĩ[3] - Ĩ[4]; Ĩ[5] - Ĩ[6]]

    return ŝ / norm(ŝ)
end


""" Successfully forces the normals to be parallel. Also works ok"""
function estimate_sun_vector_3(Ĩ, azi, elev)
    nxp = n(azi[1], elev[1]); nxm = n(azi[2], elev[2])
    nⱼ = (nxp .- (nxm)) / 2  # Average out non-parallel components 
    nⱼ /= norm(nⱼ)

    nyp = n(azi[3], elev[3]); nym = n(azi[4], elev[4])
    nₖ = (nyp .- (nym)) / 2  # Average out non-parallel components
    nₖ /= norm(nₖ)

    nzp = n(azi[5], elev[5]); nzm = n(azi[6], elev[6])
    nₗ = (nzp .- (nzm)) / 2  # Average out non-parallel components
    nₗ /= norm(nₗ)

    ns = [nⱼ'; nₖ'; nₗ']
    
    ŝ = ns \ [Ĩ[1] - Ĩ[2]; Ĩ[3] - Ĩ[4]; Ĩ[5] - Ĩ[6]]

    # ns = zeros(6, 3) 
    # for i = 1:6 
    #     ns[i, :] .= n(azi[i], elev[i])
    # end 

    # ŝ = (ns' * ns) \ (ns' * Ĩ)
    return ŝ / norm(ŝ)
end




function calibrate_with_gn(Ĩs, sᴮs; C₀ = ones(6), α₀ = [0.0,   pi,   pi/2, -pi/2, 0,     pi  ], ϵ₀ = [-pi/4, pi/4, 0.0,   0.0,  pi/4, -pi/4])
    """ Cost function that seeks to minimize the difference between measured current and expected current """
    function ff(p, Ĩs, sᴮs)
    
        Cs, αs, ϵs = p[1:6], p[7:12], p[13:18]
    
        ns = zeros(eltype(p), 6, 3)
        for i = 1:6 
            ns[i, :] .= n(αs[i], ϵs[i]) 
        end
    
        N = size(Ĩs, 1)
        diff = zeros(eltype(p), N)
        for i = 1:N 
            sᴮ = sᴮs[i, :]
            Ĩ  = Ĩs[i, :] ./ Cs
    
            bₜ = ns * sᴮ
            bₜ[bₜ .< 0.0] .= 0.0
    
            diff[i] = norm(Ĩ - bₜ)
        end
    
        return sum(diff)
    end
    
    """ Cost function that seeks to minimize the difference between true sun vector and estimated sun vector (Works better) """
    function ff2(p, Ĩs, sᴮs)
    
        Cs, αs, ϵs = p[1:6], p[7:12], p[13:18]
    
        N = size(Ĩs, 1)
        diff = zeros(eltype(p), N)
        for i = 1:N 
            sᴮ = sᴮs[i, :]
    
            Ĩ  = Ĩs[i, :] ./ Cs
            ŝᴮ = estimate_sun_vector(Ĩ, αs, ϵs)
    
            diff[i] = norm(get_err(sᴮ, ŝᴮ))
        end
    
        return sum(diff)
    end
    
    """ Residual function (gradient of the cost). Uses ff2 'cause its better. """
    function r(p, Ĩs, sᴮs)
        return ForwardDiff.gradient(_p -> ff2(_p, Ĩs, sᴮs), p)
    end
    
    """ Standard Gauss-Newton, courtesy of Kevin """
    function gauss_newton(x0, Ĩs, sᴮs; max_iters = 50, ls_iters = 20, verbose = true) 
    
        x = copy(x0); _r(_x) = r(_x, Ĩs, sᴮs)  # copy initial guess
    
        Ds = 0.0
        v = zeros(length(x))
    
        for i = 1:max_iters
    
            J = ForwardDiff.jacobian(_r, x)  # ∂r/∂x
    
            # J += Diagonal(ones(size(J, 1))) * 1e-15 # regularize?
    
            r = _r(x)
    
            v = -J\r # solve for Gauss-Newton step (direct, or indirect)
    
            S_k = dot(r,r) # calculate current cost
     
            α = 1.0 # step size (learning rate)
    
            # run a simple line search
            for ii = 1:ls_iters
                x_new = x + α*v
                S_new = norm(_r(x_new))^2
    
                # this could be updated for strong frank-wolfe conditions
                if S_new < S_k
                    x = copy(x_new)
                    Ds = S_k - S_new
                    break
                else
                    α /= 2
                end
    
                if ii == 20
                    @warn "line search failed"
                    Ds = 0
                end
            end
    
            # depending on problems caling, termination criteria should be updated
            if Ds < 1e-5
                break
            end
            if verbose
                println("\t($i)      \tJ: $(S_k) \t α: $α \t ")
            end
        end
        return x
    end






    x₀ = [C₀[:]; α₀[:]; ϵ₀[:]]
    xf = gauss_newton(x₀, Ĩs, sᴮs)

    return xf[1:6], xf[7:12], xf[13:18]
end

function calibrate_with_lls(Ĩs, sᴮs; min_current = 100.0)
    """ 
          System for calibrating the diode parameters using linear least squares, for 
        the quation  
            
            dot(n(αᵢ, ϵᵢ), sᴮ) = Ĩᵢ / Cᵢ,

        where αᵢ and ϵᵢ are the angles parameterizing the surface normal of the ith diode, 
        Iᵢ is the measured current of the ith diode, and Cᵢ is the calibration factor.  

        Note that this sets up an equation of the form `Ax = b`, where in our case 
        `b` is a vector of all measured currents, `x` is a vector of the parameters to 
        be estimated, and `A` is a block diagonal with the sun vector along the diagonal.

        Because only positive currents are allowed, this system is not truly linear. To solve 
        this, for rows with zero measured current we set the corresponding row in `A` to zero 
        so as to not confuse the solver.

        When solving this, we must multiply the calibration factor Cᵢ on the entire surface 
        normal vector, so that the vector `x` is actually [Cᵢx₁ᵢ; Cᵢy₁ᵢ; Cᵢz₁ᵢ; ...], but 
        because surface normals are unit, we can estimate C afterwords by looking at the norm.
    """
    function cart2sph(x, y, z)
        @assert (norm([x, y, z]) ≈ 1.0) "Not norm!"
        α = atan.(y, x)
        θ = atan.(sqrt.( (x.^2) .+ (y.^2)), z)  # Assumes unit 
        ϵ = pi/2 .- θ
        return α, ϵ
    end


    # Initialize matrices and set up indexing
    N = size(Ĩs, 1)
    A = zeros(6 * N, 18);
    b = zeros(6 * N);

    cids(j) = (3 * j - 2):(3 * j);
    rids(j) = (6 * j - 5):(6 * j);
    
    for i = 1:N 
        sᴮ = sᴮs[i, :]
        Ĩ  = Ĩs[i, :]
        Ai = zeros(6, 18)
    
        for j = 1:6 
            Ai[j, cids(j)] .= (Ĩ[j] > min_current)  ?  sᴮ  : zeros(3)   # Because we cant produce negative current and don't want to confuse 
        end
    
        b[rids(i)] .= Ĩ
        A[rids(i), :] .= Ai
    end


    ρ = (A' * A) \ (A' * b);  # Calculate parameters 
    Cs, αs, ϵs = zeros(6), zeros(6), zeros(6)

    for i = 1:6 
        Cs[i] = norm(ρ[cids(i)]);
        αs[i], ϵs[i] = cart2sph(   (ρ[cids(i)] / Cs[i])... );  # Normalize and then convert to spherical
    end

    return Cs, αs, ϵs #DIODES(SVector{6, Float64}(calibs), SVector{6, Float64}(azis), SVector{6, Float64}(elevs));
end

function normal_changes(α1, ϵ1, α2 = [0.0,   pi,   pi/2, -pi/2, 0,     pi  ],
                         ϵ2 = [-pi/4, pi/4, 0.0,   0.0,  pi/4, -pi/4])
    """ 
          Used to determine the angular distance from two sets of surface 
        normals, parameterized in azimuth and elevation. Defaults to comparing 
        against the standard DIODES setup.
    """
    N = size(α1, 1);
    @assert (size(α2, 1) == N) "Sets of angles must be the same length!"

    n₁ = [n(α1[i], ϵ1[i]) for i = 1:N]
    n₂ = [n(α2[i], ϵ2[i]) for i = 1:N]

    return [get_err(n₁[i], n₂[i]) for i = 1:N] # In degrees
end

function estimation_error(Ĩs, sᴮs, Cs, αs, ϵs; f = estimate_sun_vector)
    N   = size(Ĩs, 1)
    ŝ   = zeros(N, 3)
    err = zeros(N)

    for i = 1:N 
        ŝ[i, :] .= f(Ĩs[i, :] ./ Cs, αs, ϵs)    # Predict
        err[i]   = get_err(sᴮs[i, :], ŝ[i, :] ) # Get error
    end

    return ŝ, err
end

function compare_estimation(Ĩs, sᴮs; show_plots = false, Cs_uncal = ones(6), αs_uncal = [0.0, pi, pi/2, -pi/2, 0, pi], ϵs_uncal = [-pi/4, pi/4, 0.0,   0.0,  pi/4, -pi/4])

    Cs_cal, αs_cal, ϵs_cal = calibrate_with_lls(Ĩs, sᴮs)
    # Cs_cal, αs_cal, ϵs_cal = calibrate_with_gn(Ĩs, sᴮs)
    changes = normal_changes(αs_cal, ϵs_cal, αs_uncal, ϵs_uncal)

    ŝ_uncal, err_uncal = estimation_error(Ĩs, sᴮs, Cs_uncal, αs_uncal, ϵs_uncal)
    ŝ_cal, err_cal     = estimation_error(Ĩs, sᴮs, Cs_cal, αs_cal, ϵs_cal)

    μ_cal, μ_uncal = round(mean(err_cal), digits = 3),  round(mean(err_uncal), digits = 3) 
    σ_cal, σ_uncal = round(std(err_cal),  digits = 3),  round(std(err_uncal),  digits = 3) 

    if show_plots
        plot(sᴮs, title = "Sun Vector Estimates", label = false, c = [:red :blue :green], ylim = [-1, 1]);
        plot!(ŝ_uncal, label = "Uncal", c = [:red :blue :green], ls = :dot);
        display( plot!(ŝ_cal, label = "Cal", c = [:red :blue :green], ls = :dash));

        h1 = histogram(err_uncal, label = "Uncalibrated", bins = 0:40)
        h2 = histogram(err_cal, label = "Calibrated", bins = 0:40)
        display(plot(h1, h2, layout = (1, 2)))
    end

    return changes, err_uncal, err_cal, μ_uncal, μ_cal
end




@load "data/day2_combined.jld2"
idxs = [norm(Ĩs[i, :]) != 0 for i = 1:size(Ĩs, 1)];    # Trim out all the zeros
Ĩs   = Ĩs[idxs, :];  
sᴮs  = sᴮs[idxs, :];

c, eu, ec, μu, μc = compare_estimation(Ĩs, sᴮs)


# ##### Stitch together states 


# # for i = 1:7
# #     dict = load("data/segment$i.jld2")

# #     if i == 1
# #         Ĩs  = dict["Ĩs"];
# #         sᴮs = dict["sᴮs"];
# #         xs  = dict["xs"];
# #         ts  = dict["ts"];
# #     else
# #         Ĩs  = vcat(Ĩs, dict["Ĩs"]);
# #         sᴮs = vcat(sᴮs, dict["sᴮs"]);
# #         xs  = vcat(xs, dict["xs"]);
# #         ts  = vcat(ts, dict["ts"]);
# #     end;

# # end
# # @save "data/combined.jld2" Ĩs=Ĩs sᴮs=sᴮs xs=xs ts=ts

# print("done")

