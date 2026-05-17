% --- CONFIGURACIÓN ---
base_dir     = fullfile(fileparts(mfilename('fullpath')), '..', 'models');
scale_factor = 1000;   % metros → mm  (usar 1 si ya están en mm)

designs = {'CAD', 'CAD2', 'CAD_NO_BOXES'};

stl_files = {
    'link0.stl', ...
    'link1.stl', ...
    'link2.stl', ...
    'link3.stl', ...
    'link4.stl', ...
    'link5.stl', ...
    'link6.stl'
    };

% --- PROCESAMIENTO ---
total = 0;

for d = 1:length(designs)
    
    input_folder  = fullfile(base_dir, designs{d}, 'originales');
    output_folder = fullfile(base_dir, designs{d});
    
    if ~exist(input_folder, 'dir')
        fprintf('Carpeta no encontrada, saltando: %s\n', input_folder);
        continue;
    end
    
    if ~exist(output_folder, 'dir')
        mkdir(output_folder);
    end
    
    fprintf('\n=== Procesando diseño: %s ===\n', designs{d});
    
    for i = 1:length(stl_files)
        filename    = stl_files{i};
        input_path  = fullfile(input_folder,  filename);
        output_path = fullfile(output_folder, filename);
        
        if ~exist(input_path, 'file')
            fprintf('  Archivo no encontrado, saltando: %s\n', filename);
            continue;
        end
        
        fprintf('  Procesando: %s\n', filename);
        
        TR = stlread(input_path);
        vertices_scaled = TR.Points / scale_factor;
        stlwrite(triangulation(TR.ConnectivityList, vertices_scaled), output_path);
        
        fprintf('    Guardado en: %s\n', output_path);
        total = total + 1;
    end
    
end

fprintf('\nListo! %d archivos escalados.\n', total);
