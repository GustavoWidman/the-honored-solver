use std::sync::Arc;

use futures::StreamExt;
use r2r::{
    Client, Node, QosProfile, Result as R2RResult, WrappedServiceTypeSupport,
    cg_interfaces::{
        msg::RobotSensors,
        srv::{GetMap, MoveCmd, Reset},
    },
};
use tokio::sync::broadcast;

use super::types::{MoveDirection, SensorsStates};

pub struct ROSInterface {
    get_map_client: Client<GetMap::Service>,
    move_client: Client<MoveCmd::Service>,
    reset_client: Client<Reset::Service>,
    sensor_tx: broadcast::Sender<SensorsStates>,
}

impl ROSInterface {
    pub fn new(node: &mut Node) -> eyre::Result<Arc<Self>> {
        let get_map_client = create_client::<GetMap::Service>(node, "/get_map")?;
        let move_client = create_client::<MoveCmd::Service>(node, "/move_command")?;
        let reset_client = create_client::<Reset::Service>(node, "/reset")?;
        let mut sensors_subscriber =
            node.subscribe::<RobotSensors>("/culling_games/robot_sensors", QosProfile::default())?;

        let (sensor_tx, _) = broadcast::channel(100);

        let interface = Arc::new(Self {
            get_map_client,
            move_client,
            reset_client,
            sensor_tx,
        });

        let clone = interface.clone();
        tokio::task::spawn(async move {
            log::debug!("sensor subscriber started");
            loop {
                match sensors_subscriber.next().await {
                    Some(data) => {
                        if let Err(e) = clone.sensor_tx.send(data.into()) {
                            if clone.sensor_tx.receiver_count() > 0 {
                                log::warn!("failed to send sensor data: {}", e);
                            }
                            break;
                        }
                    }
                    None => {
                        log::warn!("sensor subscriber stream ended");
                        break;
                    }
                }
            }
        });

        Ok(interface)
    }

    pub async fn init(&self) -> eyre::Result<()> {
        wait_client(&self.get_map_client).await?;
        wait_client(&self.move_client).await?;
        wait_client(&self.reset_client).await?;
        Ok(())
    }

    pub async fn get_map(&self) -> eyre::Result<GetMap::Response> {
        let response = self
            .get_map_client
            .request(&GetMap::Request::default())?
            .await?;
        Ok(response)
    }

    pub async fn move_cmd(&self, direction: MoveDirection) -> eyre::Result<MoveCmd::Response> {
        let response = self
            .move_client
            .request(&MoveCmd::Request {
                direction: direction.as_str().to_string(),
            })?
            .await?;
        Ok(response)
    }

    pub async fn reset(&self, is_random: bool, map_name: String) -> eyre::Result<Reset::Response> {
        let response = self
            .reset_client
            .request(&Reset::Request {
                is_random,
                map_name,
            })?
            .await?;
        Ok(response)
    }

    pub fn subscribe_sensors(&self) -> broadcast::Receiver<SensorsStates> {
        self.sensor_tx.subscribe()
    }
}

// Helper functions
fn create_client<T: WrappedServiceTypeSupport + 'static>(
    node: &mut Node,
    service_name: &str,
) -> R2RResult<Client<T>> {
    node.create_client::<T>(service_name, QosProfile::default())
}

async fn wait_client<T: WrappedServiceTypeSupport + 'static>(client: &Client<T>) -> R2RResult<()> {
    r2r::Node::is_available(client)?.await
}
