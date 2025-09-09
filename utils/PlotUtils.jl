module PlotUtils
using Plots

# バックエンドをGRに設定（安定性のため）
gr()

export plot_contour!

function plot_contour!(plt, x, y, z, title, levels)
    # Define custom color gradient: low=blue, mid=green, high=yellow
    custom_cmap = cgrad([:blue, :green, :yellow], [0.0, 0.5, 1.0])

    # Define custom colorbar ticks
    z_min, z_max = minimum(z), maximum(z)
    colorbar_ticks = [-1.0, -0.5, 0.0, 0.25]

    # Transpose z to match contour!(x, y, z) expectation: z should be (length(x), length(y))
    contour!(plt, x, y, transpose(z),
             title=title,
             levels=levels,
             fill=true,
             c=custom_cmap,
             linecolor=:black,
             linewidth=1,
             xlabel="PowerCtrl",
             ylabel="CL",
             colorbar_title="",  # カラーバーのタイトルを消去
             colorbar_ticks=colorbar_ticks,
             colorbar_tickfontsize=12,
             colorbar_width=20,
             colorbar_titlefont=font(12, halign=:right),
             colorbar_margin=10Plots.mm,
             size=(800, 600),
             margin=5Plots.mm)
end

end
