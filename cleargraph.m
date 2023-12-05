function plotsect = cleargraph(menu,plotsect,PC,CL)

    subwindow(1,plotsect);
    grid
    title(menu);
    axis([PC(1),PC(length(PC)),CL(1),CL(length(CL))],'normal');
    gset xlabel 'Power controller input'
    gset ylabel 'CL'
    gset key top left

    plotsect = 3-plotsect;

endfunction

