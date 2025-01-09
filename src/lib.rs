#![no_std]

pub mod cdcg;

pub fn init(config: Config) {

}

#[non_exhaustive]
#[derive(Debug, Clone, Default)]
pub struct Config {
    cdcg: cdcg::Config,
}
