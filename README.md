# Manchester Challenge

## Crear el contenedor por primera vez:

<div style="border: 1px solid #5bc0de; background-color: #eaf8fc; padding: 10px; border-radius: 5px;">
  <strong>Nota:</strong> Recordar tener instalado docker-compose. 
</div>

>

Correr el siguiente comando en terminal, en el directorio del repositorio.

```bash
docker-compose up --build
```

Esto creara tanto una imagen llamada **puzzlebot_image** y tambien creara un contenedor llamado **puzzlebot_container** listo para usarse.

Puedes usar el siguiente comando para **borrar** el contenedor con docker-compose.

```bash
docker-compose down
```


### Comandos para usar el contenedor

Para **iniciar** el contenedor en futuras ocasiones.

```bash
docker start puzzlebot_container
```

Para **acceder** al contenedor en futuras ocasiones.

```bash
docker exec -it puzzlebot_container bash
```


Para **detener** el contenedor en futuras ocasiones.

```bash
docker stop puzzlebot_container
```

Para **eliminar** el contenedor en futuras ocasiones.

```bash
docker rm puzzlebot_container
```
