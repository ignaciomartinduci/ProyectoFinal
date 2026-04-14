% --- CONFIGURACIÓN ---
input_folder  = 'C:\Users\Usuario\Desktop\FING\Proyecto Final\matlab\models\CAD\originales';
output_folder = 'C:\Users\Usuario\Desktop\FING\Proyecto Final\matlab\models\CAD';
scale_factor  = 1000;   % metros → mm  (si tus STL están en metros)
                        % usar 1 si ya están en mm

% Lista de archivos a escalar (ponés los nombres de tus 3 cuerpos)
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
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

for i = 1:length(stl_files)
    filename = stl_files{i};
    input_path  = fullfile(input_folder,  filename);
    output_path = fullfile(output_folder, filename);

    fprintf('Procesando: %s\n', filename);

    % Leer STL
    TR = stlread(input_path);

    % Escalar vértices
    vertices_scaled = TR.Points / scale_factor;

    % Escribir STL escalado
    stlwrite(triangulation(TR.ConnectivityList, vertices_scaled), output_path);

    fprintf('  Guardado en: %s\n', output_path);
end

fprintf('\nListo! %d archivos escalados.\n', length(stl_files));